/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mps_voxels/JaccardMatch.h"
#include "mps_voxels/AABB.h"
#include "mps_voxels/util/hungarian.hpp"
#include <boost/bimap.hpp>
#include <utility>

namespace mps
{

JaccardMatch::JaccardMatch(const cv::Mat& labels1, const cv::Mat& labels2)
{

	if (labels1.channels() > 1 || labels1.type() != CV_16U || labels2.channels() > 1 || labels2.type() != CV_16U)
	{
		throw std::logic_error("jaccard() !!! Only works with CV_16U 1-channel Mats");
	}

	boxes1 = getBBoxes(labels1);
	boxes2 = getBBoxes(labels2);

	int count = 0;
	for (const auto& i : boxes1) { lblIndex1.insert({i.first, count++}); }
	count = 0;
	for (const auto& j : boxes2) { lblIndex2.insert({j.first, count++}); }

	intersection = Eigen::MatrixXi::Zero(boxes1.size(), boxes2.size());
	size_t dSize = std::max(boxes1.size(), boxes2.size());
	IOU = Eigen::MatrixXd::Zero(dSize, dSize);

	for (const auto& i : boxes1)
	{
		cv::Mat mask1 = (labels1 == i.first);
		int count1 = cv::countNonZero(mask1);
		iSizes.insert({i.first, count1});
		for (const auto& j : boxes2)
		{
			int intersectionCount = 0;
			double iou = 0;
			// Prune actual comparisons by starting with bounding boxes
			if (intersect(i.second, j.second))
			{
				cv::Mat mask2 = (labels2 == j.first);

				int count2;
				const auto iter = jSizes.find(j.first);
				if (iter == jSizes.end())
				{
					count2 = cv::countNonZero(mask2);
					jSizes.insert({j.first, count2});
				}
				else
				{
					count2 = iter->second;
				}

				cv::Mat intersectionMask = (mask1 & mask2);

				intersectionCount = cv::countNonZero(intersectionMask);
				int unionCount = count1 + count2 - intersectionCount;
//			    cv::Mat unionMask = (mask1 | mask2); // Optimized to be (|A|+|B|-|AB|)
				iou = (intersectionCount) / static_cast<double>(unionCount);
			}
			IOU(lblIndex1.left.at(i.first), lblIndex2.left.at(j.first)) = iou;
			intersection(lblIndex1.left.at(i.first), lblIndex2.left.at(j.first)) = intersectionCount;
		}
	}

	// If an element in J doesn't intersect anything, it won't have a size computed and cached
	for (const auto& j : boxes2)
	{
		const auto iter = jSizes.find(j.first);
		if (iter == jSizes.end())
		{
			cv::Mat mask2 = (labels2 == j.first);
			int count2 = cv::countNonZero(mask2);
			jSizes.insert({j.first, count2});
		}
	}

	// Compute the optimal assignment of patches
	Hungarian H(-IOU);
	const auto& A = H.getAssignment();
	boost::bimap<LabelT, LabelT> matches;
	for (size_t i = 0; i < A.size(); ++i)
	{
		if (i >= boxes1.size()) { continue; }
		if (A[i] >= (int)boxes2.size()) { continue; }
		matches.insert({lblIndex1.right.at(i), lblIndex2.right.at(A[i])});
	}

	match = {-H.getSolutionCost(), matches};
}

double JaccardMatch::symmetricCover() const
{
	double score = 0.0;
	Eigen::VectorXd iMax = IOU.rowwise().maxCoeff();
	for (const auto& pair : lblIndex1.left)
	{
		// pair is (label, index)
		score += iSizes.at(pair.first) * iMax[pair.second];
	}
	Eigen::RowVectorXd jMax = IOU.colwise().maxCoeff();
	for (const auto& pair : lblIndex2.left)
	{
		// pair is (label, index)
		score += jSizes.at(pair.first) * jMax[pair.second];
	}

	double fullSize = 0;
	for (const auto& i : iSizes) { fullSize += i.second; }
	return score / (2.0 * fullSize);
}

JaccardMatch3D::LabelBounds
getBBoxes(const OccupancyData& labels)
{
	JaccardMatch3D::LabelBounds boxes;
	if (!labels.objects.empty())
	{
		for (const auto& kv : labels.objects)
		{
			boxes.emplace(kv.first.id, JaccardMatch3D::AABB(kv.second->minExtent, kv.second->maxExtent));
		}
	}
	else
	{
		for (size_t i = 0; i < labels.voxelRegion->num_vertices(); ++i)
		{
			const auto val = labels.vertexState[i];
			if (val != mps::VoxelRegion::FREE_SPACE)
			{
				const auto coord = labels.voxelRegion->coordinate_of(labels.voxelRegion->vertex_at(i));
				boxes[val].extend(coord);
			}
		}
	}
	return boxes;
}

std::set<size_t>
sparse_matches(const VoxelRegion::VertexLabels& labels,
               const VoxelRegion::VertexLabels::value_type & id)
{
	std::set<size_t> res;
	for (size_t i = 0; i < labels.size(); ++i)
	{
		if (labels[i] == id) { res.insert(i); }
	}
	return res;
}

JaccardMatch3D::JaccardMatch3D(const OccupancyData& labels1, const OccupancyData& labels2)
{
//	if (labels1.channels() > 1 || labels1.type() != CV_16U || labels2.channels() > 1 || labels2.type() != CV_16U)
//	{
//		throw std::logic_error("jaccard() !!! Only works with CV_16U 1-channel Mats");
//	}
	assert(labels1.voxelRegion->num_vertices() == labels1.vertexState.size());
	assert(labels2.voxelRegion->num_vertices() == labels2.vertexState.size());
	assert(labels1.voxelRegion->num_vertices() == labels2.voxelRegion->num_vertices());

	boxes1 = getBBoxes(labels1);
	boxes2 = getBBoxes(labels2);

	int count = 0;
	for (const auto& i : boxes1) { lblIndex1.insert({i.first, count++}); }
	count = 0;
	for (const auto& j : boxes2) { lblIndex2.insert({j.first, count++}); }

	intersection = Eigen::MatrixXi::Zero(boxes1.size(), boxes2.size());
	size_t dSize = std::max(boxes1.size(), boxes2.size());
	IOU = Eigen::MatrixXd::Zero(dSize, dSize);

	for (const auto& i : boxes1)
	{
		const auto mask1 = sparse_matches(labels1.vertexState, i.first);
		int count1 = mask1.size();
		iSizes.insert({i.first, count1});
		for (const auto& j : boxes2)
		{
			int intersectionCount = 0;
			double iou = 0;
			// Prune actual comparisons by starting with bounding boxes
			if (i.second.intersects(j.second))
			{
				const auto mask2 = sparse_matches(labels2.vertexState, j.first);

				int count2;
				const auto iter = jSizes.find(j.first);
				if (iter == jSizes.end())
				{
					count2 = mask2.size();
					jSizes.insert({j.first, count2});
				}
				else
				{
					count2 = iter->second;
				}

				std::vector<size_t> intersectionMask;
				std::vector<int> crossover;
				std::set_intersection(mask1.begin(), mask1.end(), mask2.begin(), mask2.end(), std::back_inserter(intersectionMask));

				intersectionCount = intersectionMask.size();
				int unionCount = count1 + count2 - intersectionCount;
//			    cv::Mat unionMask = (mask1 | mask2); // Optimized to be (|A|+|B|-|AB|)
				iou = (intersectionCount) / static_cast<double>(unionCount);
			}
			IOU(lblIndex1.left.at(i.first), lblIndex2.left.at(j.first)) = iou;
			intersection(lblIndex1.left.at(i.first), lblIndex2.left.at(j.first)) = intersectionCount;
		}
	}

	// If an element in J doesn't intersect anything, it won't have a size computed and cached
	for (const auto& j : boxes2)
	{
		const auto iter = jSizes.find(j.first);
		if (iter == jSizes.end())
		{
			const auto mask2 = sparse_matches(labels2.vertexState, j.first);
			int count2 = mask2.size();
			jSizes.insert({j.first, count2});
		}
	}

	// Compute the optimal assignment of patches
	Hungarian H(-IOU);
	const auto& A = H.getAssignment();
	boost::bimap<LabelT, LabelT> matches;
	for (size_t i = 0; i < A.size(); ++i)
	{
		if (i >= boxes1.size()) { continue; }
		if (A[i] >= (int)boxes2.size()) { continue; }
		matches.insert({lblIndex1.right.at(i), lblIndex2.right.at(A[i])});
	}

	match = {-H.getSolutionCost(), matches};
}

double JaccardMatch3D::symmetricCover() const
{
	double score = 0.0;
	Eigen::VectorXd iMax = IOU.rowwise().maxCoeff();
	for (const auto& pair : lblIndex1.left)
	{
		// pair is (label, index)
		score += iSizes.at(pair.first) * iMax[pair.second];
	}
	Eigen::RowVectorXd jMax = IOU.colwise().maxCoeff();
	for (const auto& pair : lblIndex2.left)
	{
		// pair is (label, index)
		score += jSizes.at(pair.first) * jMax[pair.second];
	}

	double fullSize = 0;
	for (const auto& i : iSizes) { fullSize += i.second; }
	return score / (2.0 * fullSize);
}

}
