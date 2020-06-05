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

#ifndef MPS_RELABEL_TREE_IMAGE_HPP
#define MPS_RELABEL_TREE_IMAGE_HPP

#include "mps_voxels/ValueTree_impl.hpp"
#include "mps_voxels/image_utils.h"

namespace mps
{

template <typename Map, typename ValueTree>
cv::Mat relabelCut(const ValueTree& T, const tree::TreeCut& cut, const Map& treeIndexToImageLabel, const cv::Mat& oldLabels)
{
	// Create a map from current label to new label, then call the parallelized relabel function
	std::map<uint16_t, uint16_t> labelTransform;
	uint16_t newLabel = 0;

	for (const auto node : cut)
	{
		++newLabel;
		std::set<tree::NodeID> children;
		tree::descendants(T, node, children);
		children.insert(node); // This cut node could be a leaf as well
		for (const auto c : children)
		{
			if (tree::children(T, c).empty()) // Leaf node
			{
				labelTransform.insert({treeIndexToImageLabel.at(c), newLabel});
			}
		}
	}

	return relabel(oldLabels, labelTransform);
}

}
#endif // MPS_RELABEL_TREE_IMAGE_HPP
