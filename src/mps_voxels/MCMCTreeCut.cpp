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

#include "mps_voxels/MCMCTreeCut.h"
#include "mps_voxels/MCMCTreeCut.hpp"
#include "mps_voxels/ValueTree.hpp"

namespace mps
{

namespace tree
{

template class MCMCTreeCut<DenseValueTree>;
template class MCMCTreeCut<SparseValueTree>;


DenseValueTree test_tree_1()
{
	DenseValueTree T;
	T.parent_ = {0, 0, 0, 1, 1, 2, 2, 6, 6, 6 };
	T.children_ = {{1, 2}, {3, 4}, {5, 6}, {}, {}, {}, {7, 8, 9}, {}, {}, {}};
	T.value_ = {1.0, 5.0, 3.0, 2.0, 2.0, 2.0, 6.0, 1.0, 2.0, 1.0};

	return T;
}

void test_optimal_cut()
{
	DenseValueTree T = test_tree_1();

	assert(root(T) == 0);
	assert(T.parent_.size() == T.children_.size());
	assert(T.parent_.size() == T.value_.size());

	auto opt = optimalCut(T);
	assert(opt.first == 13);
	assert(opt.second.size() == 3);

	// Test returning
	MoveCut m{1, false};
	auto res = apply(T, opt.second, m);
	assert(value(T, res) < opt.first);
	assert(res.size() == 4);

	MoveCut m2{3, true};
	res = apply(T, res, m2);
	assert(res == opt.second);
}

}

}
