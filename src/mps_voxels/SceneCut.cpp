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

#include "mps_voxels/SceneCut.h"

#include <vector>
#include <numeric>
#include <algorithm>

#include <iostream>
#include <opencv2/imgproc.hpp>

namespace mps
{

using namespace mps::tree;

std::vector<double> computeAreas(const Ultrametric& um, const DenseValueTree& vt, const cv::Mat& labels)
{
	std::vector<double> areas(um.merge_tree.start_ths.size(), 0.0);

	for (size_t lbl = 0; lbl <= um.merge_tree.n_leaves; ++lbl)
	{
//		areas[lbl] = stats2.at<int>(lbl, cv::CC_STAT_AREA) / 4;
		auto iter = um.label_to_index.find(lbl); // (label, index)
		if (iter != um.label_to_index.end())
		{
			areas[iter->second] = cv::countNonZero(labels == lbl);
		}
	}

	for (size_t node = 0; node < areas.size(); ++node)
	{
		for (auto c : children(vt, node))
		{
			assert(areas[c] != 0);
			areas[node] += areas[c];
		}
	}

	return areas;
}

DenseValueTree SceneCut::process(const Ultrametric& um, const cv::Mat& labels) const
{
	DenseValueTree vt;
	const auto numNodes = um.merge_tree.start_ths.size();
	vt.parent_.resize(numNodes);
	std::iota(vt.parent_.begin(), vt.parent_.end(), 0);
	vt.children_.resize(numNodes);
	vt.value_.resize(numNodes);

	for (const auto& pair : um.parent_tree)
	{
		int childID = um.label_to_index.at(pair.first);
		int parentID = um.label_to_index.at(pair.second);
		vt.parent_[childID] = parentID;
		vt.children_[parentID].push_back(childID);
	}

	// Validate tree
	int rootCount = 0;
	for (size_t n = 0; n < numNodes; ++n)
	{
		if (vt.parent_[n] == static_cast<int>(n))
		{
			rootCount++;
		}
	}
	if (rootCount != 1)
	{
		throw std::logic_error("Multiply-rooted tree.");
	}

	std::vector<double> areas = computeAreas(um, vt, labels);

	// Calculate node values
	for (size_t n = 0; n < numNodes; ++n)
	{
		double start_ths = um.merge_tree.start_ths[n];
		double end_ths = um.merge_tree.start_ths[vt.parent_[n]];
		if (vt.parent_[n] == static_cast<int>(n)) { end_ths = 1.0; } // upper value of root node is 1
		vt.value_[n] = parameters.prior_prob
			* std::exp(-std::pow(std::abs(start_ths /* - 0 */), 2) / (parameters.sigma_i * parameters.sigma_i))
			* std::exp(-std::pow(std::abs(end_ths - 1), 2) / (parameters.sigma_o * parameters.sigma_o));

		if (areas[n] > parameters.max_area || areas[n] < parameters.min_area)
		{
			vt.value_[n] = parameters.out_of_area_range_weight;
		}
		vt.value_[n] = std::log(vt.value_[n]) * pow(areas[n], parameters.area_exponent);
	}


	return vt;
}

}
