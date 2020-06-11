/**
 * \file VoronoiTessellator.h
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

#ifndef VORONOITESSELLATOR_H
#define VORONOITESSELLATOR_H

//#include "grasp_capture/VoronoiVisualizer.h"
#include "mps_voxels/util/samplers.h"

#include <Eigen/Geometry>
#include <Eigen/StdVector>

//#include <dart/math/Geometry.h>
#include <boost/polygon/voronoi.hpp>

#include <set>
#include <memory>

namespace mps
{
namespace tessellation
{
typedef Eigen::Vector2i Ray;
typedef Eigen::Vector2i Point;
}
}

namespace std
{
template<>
struct less<mps::tessellation::Point>
{
	bool operator()(const mps::tessellation::Point& lhs,
	                const mps::tessellation::Point& rhs) const
	{
		if (lhs.x() < rhs.x())
		{
			return true;
		}
		else if (lhs.x() > rhs.x())
		{
			return false;
		}
		else // lhs.x() == rhs.x()
		{
			if (lhs.y() < rhs.y())
			{
				return true;
			}
			else
			{
				return false;
			}
		}
	}
};
} // namespace std

namespace mps
{

using SupportPoints = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

class VoronoiTessellator
{
public:
	typedef tessellation::Point Point;
	typedef tessellation::Ray Ray;
	template<class T>
	using voronoi_diagram = boost::polygon::voronoi_diagram<T>;

	static_assert(sizeof(int) >= 4, "Int must be large enough for scale.");
	static constexpr double SCALE = 100000;

	VoronoiTessellator(const SupportPolygon& _SP);

	SupportPoints tessellate(const size_t nD);

	bool inSupport(const Point& pt) const;
	bool inSupport(const voronoi_diagram<double>::vertex_type& v) const;
	Point toFiniteEdge(const voronoi_diagram<double>::edge_type& edge) const;
	bool cleanEdge(const voronoi_diagram<double>::edge_type& edge, std::pair<Point, Point>& vertices) const;
	Point centroid(const voronoi_diagram<double>::cell_type& cell, const Point& prior) const;

	// https://en.wikipedia.org/wiki/Lloyd%27s_algorithm
	std::shared_ptr<voronoi_diagram<double>> relax(const voronoi_diagram<double>& in, std::vector<Point>& centers, long& delta);

	const voronoi_diagram<double>& getDiagram() const { return *mVD; }
	const SupportPolygon& getSupport() const { return mSP; }
	const std::vector<Point>& getCenters() const { return mCenters; }

protected:
	const SupportPolygon mSP;
	UniformPolygonRejectionSampler mSampler;
	std::vector<std::pair<Point, Point>> mFence;
	std::vector<Point> mCenters;
	std::set<Point> mFixed;
	std::shared_ptr<boost::polygon::voronoi_diagram<double>> mVD;
};

typedef std::shared_ptr<VoronoiTessellator> VoronoiTessellatorPtr;


} // namespace grasp

#endif // VORONOITESSELLATOR_H
