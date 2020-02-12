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

#ifndef MPS_MCMCTREECUT_H
#define MPS_MCMCTREECUT_H

#include "mps_voxels/Belief.hpp"
#include "mps_voxels/ValueTree.h"
#include "mps_voxels/UniformRandomSelector.hpp"

#include <tuple>
#include <cassert>

namespace mps
{

namespace tree
{

template <typename ValueTree>
class MCMCTreeCut : public Belief<TreeCut>
{
public:
	ValueTree T;
	TreeCut cStar;
	double vStar;

	uniform_random_selector<> randomSelector;
	std::uniform_real_distribution<> uni;

	double sigmaSquared;
	int nTrials = 10; // Mixing number to reduce MCMC autocorrelation

	explicit
	MCMCTreeCut(ValueTree t, const double sigSquared);

	std::pair<double, TreeCut> sample(RNG& rng, SAMPLE_TYPE type) override;

};

struct MoveCut
{
	int index;
	bool up;
};

template <typename ValueTree>
TreeCut apply(const ValueTree& T, const TreeCut& original, const MoveCut& move);

template <typename ValueTree>
std::vector<MoveCut> enumerateMoves(const ValueTree& T, const TreeCut& original);

template <typename ValueTree>
double returnProbability(const ValueTree& T, const TreeCut& original, const MoveCut& proposal);

double logProbCut(const double optVal, const double cutVal, const double sigmaSquared);

template <typename ValueTree>
std::vector<std::pair<double, TreeCut>> sampleCuts(const ValueTree& T);

DenseValueTree
test_tree_1();

void test_optimal_cut();

extern template class MCMCTreeCut<DenseValueTree>;
extern template class MCMCTreeCut<SparseValueTree>;

}

}

#endif // MPS_MCMCTREECUT_H
