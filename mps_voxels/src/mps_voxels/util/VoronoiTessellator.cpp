/**
 * \file VoronoiTessellator.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-7-9
 *
 * \copyright
 *
 * Copyright (c) 2020, Andrew Price
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mps_voxels/util/VoronoiTessellator.h"
#include "mps_voxels/util/geometry.h"

using boost::polygon::voronoi_builder;
using boost::polygon::voronoi_diagram;

// Perform concept mapping for boost templated methods
namespace boost
{
namespace polygon
{

template<>
struct geometry_concept<mps::VoronoiTessellator::Point> { typedef point_concept type; };

template<>
struct point_traits<mps::VoronoiTessellator::Point>
{
	typedef int coordinate_type;

	static inline coordinate_type get(const mps::VoronoiTessellator::Point& point, orientation_2d orient)
	{
		return (orient == HORIZONTAL) ? point.x() : point.y();
	}
};

} // namespace polygon
} // namespace boost

namespace mps
{

constexpr double VoronoiTessellator::SCALE; // Prevents linker error with -O0

VoronoiTessellator::VoronoiTessellator(const SupportPolygon& _SP)
    : mSP(_SP),
      mSampler(mSP)
{
	for (size_t j = 0; j < mSP.size(); ++j)
	{
		// Fence
		size_t k = (j+1) % mSP.size();
		Eigen::Vector2d seedA = mSP[j] * SCALE;
		Eigen::Vector2d seedB = mSP[k] * SCALE;
		mFence.push_back(std::pair<Point,Point>(Point(seedA.x(), seedA.y()),
		                                        Point(seedB.x(), seedB.y())));
	}
}

SupportPoints VoronoiTessellator::tessellate(const size_t nD)
{
	assert(nD > mSP.size());
	for (size_t i = 0; i < nD - mSP.size(); ++i)
	{
		// Get an interior point
		Eigen::Vector2d seed = mSampler.getPoint();

		seed *= SCALE;
		mCenters.emplace_back(Point(seed.x(), seed.y()));
	}

	for (size_t j = 0; j < mSP.size(); ++j)
	{
		// Seed the corners
		Eigen::Vector2d seed = mSP[j] * SCALE;
		mCenters.emplace_back(Point(seed.x(), seed.y()));
		mFixed.insert(Point(seed.x(), seed.y()));
	}

	mVD = std::make_shared<voronoi_diagram<double>>();
	construct_voronoi(mCenters.begin(), mCenters.end(), mVD.get());

	long delta = 0;

	const int nR = 50;
	for (int i = 0; i < nR; ++i)
	{
		mVD = relax(*mVD, mCenters, delta);
		if (0 == delta)
		{
			break;
		}
	}

	SupportPoints centersOut;
	for (const Point& p : mCenters)
	{
		centersOut.push_back({static_cast<double>(p.x())/SCALE,
		                      static_cast<double>(p.y())/SCALE});
	}
	return centersOut;
}

bool VoronoiTessellator::inSupport(const VoronoiTessellator::Point& pt) const
{
	if (mFixed.end() != mFixed.find(pt))
	{
		return true;
	}

	Eigen::Vector2d pnt(pt.x(), pt.y());
	pnt /= SCALE;

	return pnpoly(mSP, pnt);
}

bool VoronoiTessellator::inSupport(const voronoi_diagram<double>::vertex_type& v) const
{
	return inSupport(Point(v.x(), v.y()));
}

VoronoiTessellator::Point VoronoiTessellator::toFiniteEdge(const voronoi_diagram<double>::edge_type& edge) const
{
	Point p1 = mCenters[edge.cell()->source_index()];
	Point p2 = mCenters[edge.twin()->cell()->source_index()];

	Point o((p1 + p2) / 2);

	Ray ray(-(p2.y() - p1.y()), (p2.x() - p1.x()));

	std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>> xs;

	// Don't know correct sign of ray, so have to try all edges/signs and find the min...
	for (size_t i = 0; i < mFence.size(); ++i)
	{
		Eigen::Vector2f x = Eigen::Vector2f::Zero();
		if (rayLineSegmentIntersection<float>(o.cast<float>(), ray.cast<float>(),
		                                      mFence[i].first.cast<float>(), mFence[i].second.cast<float>(), x))
		{
			xs.push_back(x);
		}
		else if (rayLineSegmentIntersection<float>(o.cast<float>(), -ray.cast<float>(),
		                                           mFence[i].first.cast<float>(), mFence[i].second.cast<float>(), x))
		{
			xs.push_back(x);
		}
	}

	assert(!xs.empty());
	assert(xs.size() <= 3);

	Eigen::Vector2f xFinal = xs.front();
	for (const Eigen::Vector2f& x : xs)
	{
		if ((x-o.cast<float>()).norm() < (xFinal-o.cast<float>()).norm())
		{
			xFinal = x;
		}
	}

	return xFinal.cast<int>();
}

bool VoronoiTessellator::cleanEdge(const voronoi_diagram<double>::edge_type& edge, std::pair<VoronoiTessellator::Point, VoronoiTessellator::Point>& vertices) const
{
	bool in0 = (edge.vertex0()) && inSupport(*edge.vertex0());
	bool in1 = (edge.vertex1()) && inSupport(*edge.vertex1());

	if (in0 && in1)
	{
		// Both vertices exist and in polygon
		vertices = std::pair<Point,Point>(Point(edge.vertex0()->x(),edge.vertex0()->y()),
		                                  Point(edge.vertex1()->x(),edge.vertex1()->y()));
		return true;
	}
	else if (!in0 && !in1)
	{
		// Both vertices invalid
		vertices = std::pair<Point,Point>(Point(NAN,NAN), Point(NAN,NAN));
		return false;
	}
	else
	{
		// One valid vertex
		const voronoi_diagram<double>::vertex_type* v = (in0) ? edge.vertex0() : edge.vertex1();

		Point x = toFiniteEdge(edge);
		vertices = std::pair<Point,Point>(Point(v->x(),v->y()),x);
		return true;
	}
}

VoronoiTessellator::Point VoronoiTessellator::centroid(const voronoi_diagram<double>::cell_type& cell, const VoronoiTessellator::Point& prior) const
{
	const voronoi_diagram<double>::edge_type* edge = cell.incident_edge();

	std::set<Point> vertices;
	int count = 0;

	if (nullptr == edge)
	{
		return prior;
	} // WTF?
	do
	{
		std::pair<Point,Point> endpoints;
		if (cleanEdge(*edge, endpoints))
		{
			vertices.insert(endpoints.first);
			vertices.insert(endpoints.second);
		}

		edge = edge->next();
		++count;
	} while (edge != cell.incident_edge());

	Point retVal = Point::Zero();
	for (const Point& vertex : vertices)
	{
		retVal += vertex / static_cast<long>(vertices.size()); // Division happens inside loop to prevent overflow
	}

	return retVal;
}

std::shared_ptr<voronoi_diagram<double> > VoronoiTessellator::relax(const voronoi_diagram<double>& in, std::vector<VoronoiTessellator::Point>& centers, long& delta)
{
	delta = 0;
	for (const voronoi_diagram<double>::cell_type& cell : in.cells())
	{
		size_t idx = cell.source_index();
		if (mFixed.end() == mFixed.find(centers[idx]))
		{
			Point newPt = centroid(cell, centers[idx]);
			if (inSupport(newPt))
			{
				delta += (centers[idx] - newPt).norm();
				centers[idx] = newPt;
			}
		}
	}

	std::shared_ptr<voronoi_diagram<double>> out = std::make_shared<voronoi_diagram<double>>();
	construct_voronoi(centers.begin(), centers.end(), out.get());

	return out;
}


} // namespace grasp
