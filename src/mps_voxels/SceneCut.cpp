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

namespace mps
{

ValueTree SceneCut::process(const Ultrametric& um) const
{
	ValueTree vt;
	const auto numNodes = um.merge_tree.start_ths.size();
	vt.parent.resize(numNodes);
	std::iota(vt.parent.begin(), vt.parent.end(), 0);
	vt.children.resize(numNodes);
	vt.value.resize(numNodes);

	for (const auto& pair : um.parent_tree)
	{
		int childID = um.label_to_index.at(pair.first);
		int parentID = um.label_to_index.at(pair.second);
		vt.parent[childID] = parentID;
		vt.children[parentID].push_back(childID);
	}

	for (size_t n = 0; n < numNodes; ++n)
	{
		double start_ths = um.merge_tree.start_ths[n];
		double end_ths = um.merge_tree.start_ths[vt.parent[n]];
		vt.value[n] = parameters.prior_prob
			* std::exp(-std::pow(std::abs(start_ths /* - 0 */), 2) / (parameters.sigma_i * parameters.sigma_i))
			* std::exp(-std::pow(std::abs(end_ths - 1), 2) / (parameters.sigma_o * parameters.sigma_o));
	}

	// Validate tree
	int rootCount = 0;
	for (size_t n = 0; n < numNodes; ++n)
	{
		if (vt.parent[n] == static_cast<int>(n))
		{
			rootCount++;
		}
	}
	if (rootCount != 1)
	{
		throw std::logic_error("Multiply-rooted tree.");
	}


	return vt;
}

}
