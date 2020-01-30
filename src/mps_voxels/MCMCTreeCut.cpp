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
#include "mps_voxels/UniformRandomSelector.hpp"

#include <algorithm>
#include <random>

namespace mps
{

TreeCut apply(const ValueTree& T, const TreeCut& original, const MoveCut& move)
{
	TreeCut newCut = original;
	newCut.erase(move.index);

	if (move.up)
	{
		newCut.insert(T.parent[move.index]);

		// Prune any orphaned cuts
		TreeCut orphans;
		for (const int n : newCut)
		{
			// Check if this node has any ancestors in the cut
			auto A = ancestors(T, n);
			std::vector<int> intersection;
			std::set_intersection(newCut.begin(), newCut.end(),
			                      A.begin(), A.end(),
			                      std::back_inserter(intersection));
			if (!intersection.empty())
			{
				orphans.insert(n);
			}
		}

		for (const int n : orphans)
		{
			newCut.erase(n);
		}
	}
	else
	{
		const auto& children = T.children[move.index];
		for (const int c : children)
		{
			newCut.insert(c);
		}
	}
	return newCut;
}

std::vector<MoveCut> enumerateMoves(const ValueTree& T, const TreeCut& original)
{
	std::vector<MoveCut> moves;
	for (const int n : original)
	{
		if (T.parent[n] != n)
		{
			moves.push_back({n, true});
		}
		if (!T.children[n].empty())
		{
			moves.push_back({n, false});
		}
	}
	return moves;
}

double returnProbability(const ValueTree& T, const TreeCut& original, const MoveCut& proposal)
{
	TreeCut newCut = apply(T, original, proposal);

	int returns = 0;
	std::vector<MoveCut> successors = enumerateMoves(T, newCut);
	for (const auto& m : successors)
	{
		auto C = apply(T, newCut, m);
		if (C == original)
		{
			++returns;
		}
	}

	return static_cast<double>(returns)/static_cast<double>(successors.size());
}

double logProbCut(const double optVal, const double cutVal, const double sigmaSquared)
{
	double delta = optVal - cutVal;

	return -(delta*delta)/sigmaSquared;
}

std::vector<std::pair<double, TreeCut>> sampleCuts(const ValueTree& T)
{
	std::vector<std::pair<double, TreeCut>> cuts;
	TreeCut cStar;
	double cStarVal;
	std::tie(cStarVal, cStar) = optimalCut(T);

	cuts.push_back({0.0, cStar});

	uniform_random_selector<> randomSelector;
	std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));
	std::random_device rd;
	std::default_random_engine re(rd());

	const double sigmaSquared = 10.0;
	const int nTrials = 100;
	TreeCut cut_i = cStar;
	for (int i = 0; i < nTrials; ++i)
	{
		auto moves = enumerateMoves(T, cut_i);
		auto move = randomSelector(moves);

		TreeCut cut_prime = apply(T, cut_i, move);
		double value_prime = value(T, cut_prime);

		double logprob_i = logProbCut(cStarVal, value(T, cut_i), sigmaSquared);
		double logprob_prime = logProbCut(cStarVal, value_prime, sigmaSquared);

		double proposal_prob = 1.0/static_cast<double>(moves.size());
		double return_prob = returnProbability(T, cut_i, move);

		double acceptance = std::exp(logprob_prime + std::log(return_prob) - (logprob_i + std::log(proposal_prob)));

		if (uni(rd) < acceptance)
		{
			cuts.push_back({logprob_prime, cut_prime});
			cut_i = cut_prime;
		}
	}

	return cuts;
}

ValueTree test_tree_1()
{
	ValueTree T;
	T.parent = {0, 0, 0, 1, 1, 2, 2, 6, 6, 6 };
	T.children = {{1, 2}, {3, 4}, {5, 6}, {}, {}, {}, {7, 8, 9}, {}, {}, {}};
	T.value = {1.0, 5.0, 3.0, 2.0, 2.0, 2.0, 6.0, 1.0, 2.0, 1.0};

	return T;
}

void test_optimal_cut()
{
	ValueTree T = test_tree_1();

	assert(root(T) == 0);
	assert(T.parent.size() == T.children.size());
	assert(T.parent.size() == T.value.size());

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

std::pair<double, TreeCut> MCMCTreeCut::sample(RNG& rng, const Belief<TreeCut>::SAMPLE_TYPE type)
{
	if (Belief<TreeCut>::SAMPLE_TYPE::RANDOM == type)
	{
		TreeCut cut_i = cStar;
		double logprob_i = 0.0;
		for (int i = 0; i < nTrials; ++i)
		{
			auto moves = enumerateMoves(T, cut_i);
			auto move = randomSelector(moves);

			TreeCut cut_prime = apply(T, cut_i, move);
			double value_prime = value(T, cut_prime);

//			double logprob_i = logProbCut(vStar, value(T, cut_i), sigmaSquared);
			double logprob_prime = logProbCut(vStar, value_prime, sigmaSquared);

			double proposal_prob = 1.0/static_cast<double>(moves.size());
			double return_prob = returnProbability(T, cut_i, move);

			double acceptance = std::exp(logprob_prime + std::log(return_prob) - (logprob_i + std::log(proposal_prob)));

			if (uni(rng) < acceptance)
			{
//				cuts.push_back({logprob_prime, cut_prime});
				cut_i = cut_prime;
				logprob_i = logprob_prime;
			}
		}
		return {logprob_i, cut_i};
	}
	else
	{
		auto cutStar = optimalCut(T);
		return {0.0, cutStar.second};
	}
}

MCMCTreeCut::MCMCTreeCut(ValueTree t, const double sigSquared)
	: Belief<TreeCut>(),
	  T(std::move(t)),
	  uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max())),
	  sigmaSquared(sigSquared)
{
	std::tie(vStar, cStar) = optimalCut(T);
}

}
