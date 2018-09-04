/**
 * \file Viterbi.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2018-06-02
 *
 * \copyright
 *
 * Copyright (c) 2018, Andrew Price
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

#ifndef MPS_VOXELS_VITERBI_H
#define MPS_VOXELS_VITERBI_H

#include <vector>
#include <iostream>
#include <limits>
#include <cassert>

namespace mps
{

struct CostAccumulator
{
	double cost;
	int predecessor;

	CostAccumulator()
	{
		cost = std::numeric_limits<double>::max();
		predecessor = -1;
	}
};


/**
 * @brief viterbi Computes the indices for the minimum cost path through a trellis structure
 * @tparam T Elemental type for states within the steps of the trellis
 * @param trellis
 * @param stateCostFn Function pointer or functor for evaluating the cost of a single state in the trellis.
 *  Of the form \code double StateCostFn(const T&) \endcode.
 * @param transitionCostFn Function pointer or functor for evaluating the cost of a state-action-state transition in the trellis.
 *  Of the form \code double TransitionCostFn(const T&, const T&) \endcode.
 * @return Vector of indices for the minimum cost path through a trellis structure
 */
template <class T, class A, typename StateCostFn, typename TransitionCostFn>
std::vector<int> viterbi(const std::vector<std::vector<T, A> >& trellis, const std::vector<double>& times, const StateCostFn stateCostFn, const TransitionCostFn transitionCostFn, const double maxAllowedTransitionCost = std::numeric_limits<double>::max())
{
	const size_t N = trellis.size();
	if (times.size() != N) { throw std::runtime_error("Time size mismatch: " + std::to_string(times.size()) + " vs " + std::to_string(N)); }

	std::vector<std::vector<CostAccumulator> > costGraph;
	costGraph.reserve(N);

#ifdef GENERATE_TEXT_GRAPH
	std::cout << "digraph g \n{\n\trankdir=\"LR\";\n\tsplines=\"line\";" << std::endl;
#endif

	for (size_t n = 0; n < N; ++n)
	{
		// Number of solutions in current time index
		const size_t I = trellis[n].size();
		assert(I > 0);

		std::vector<CostAccumulator> costGraphSlice;
		costGraphSlice.reserve(I);
		bool hasValidTransition = false;

		for (size_t i = 0; i < I; ++i)
		{
			CostAccumulator acc; // Accumulated cost

			double stateCost = stateCostFn(trellis[n][i]);

#ifdef GENERATE_TEXT_GRAPH
			//std::cout << "\tn" << n << "_" << i << "[label=\"" << stateCost << "\"];" << std::endl;
#endif

			if (n > 0)
			{
				// Number of solutions in previous time index
				const size_t J = trellis[n-1].size();
				assert(J > 0);
				for (size_t j = 0; j < J; ++j)
				{
					double transCost = transitionCostFn(trellis[n-1][j], times[n-1], trellis[n][i], times[n]);
					if (transCost > maxAllowedTransitionCost) { continue; }
					double netCost = costGraph[n-1][j].cost + transCost + stateCost;

#ifdef GENERATE_TEXT_GRAPH
					std::cout << "\tn" << n-1 << "_" << j << "->" << "n" << n << "_" << i << "[label=\"" << transitionCostFn(trellis[n-1][j], times[n-1], trellis[n][i], times[n]) << "\"];" << std::endl;
#endif

					if (netCost < acc.cost)
					{
						acc.cost = netCost;
						acc.predecessor = j;
						hasValidTransition = true;
					}
				}
			}
			else
			{
				// For first state, no prior transition cost
				acc.cost = stateCost;
				hasValidTransition = true;
			}
#ifdef GENERATE_TEXT_GRAPH
			std::cout << "\tn" << n << "_" << i << "[label=\"" << acc.cost << "\"];" << std::endl;
#endif
			costGraphSlice.push_back(acc);

		}

		if (!hasValidTransition)
		{
			// No solution was found that satisfied the cost.
			// (All transitions between times n-1 and n violate maxAllowedTransitionCost)
#ifdef GENERATE_TEXT_GRAPH
			std::cout << "\n}" << std::endl;
#endif
			return std::vector<int>();
		}
		costGraph.push_back(costGraphSlice);
	}

	// Get the minimum cost path
	CostAccumulator minFinalCost;
	for (size_t i = 0; i < costGraph[N-1].size(); ++i)
	{
		if (costGraph[N-1][i].cost < minFinalCost.cost)
		{
			minFinalCost.cost = costGraph[N-1][i].cost;
			minFinalCost.predecessor = i;
		}
	}
	assert(-1 != minFinalCost.predecessor);

	// Backtrack the solution
	std::vector<int> minCostPath(N);
	for (int i = N-1; i >= 0; --i)
	{
		minCostPath[i] = minFinalCost.predecessor;
		minFinalCost = costGraph[i][minFinalCost.predecessor];
	}

#ifdef GENERATE_TEXT_GRAPH
	// Bold edges of selected path
	std::cout << std::endl;
	for (size_t n = 1; n < N; ++n)
	{
		std::cout << "\tn" << n-1 << "_" << minCostPath[n-1] << "->" << "n" << n << "_" << minCostPath[n] << "[penwidth=\"3\"];" << std::endl;
	}
	std::cout << "}" << std::endl;
#endif

	return minCostPath;
}

template <class T, class A, typename StateCostFn, typename TransitionCostFn>
std::vector<int> viterbi(const std::vector<std::vector<T, A> >& trellis, const StateCostFn stateCostFn, const TransitionCostFn transitionCostFn, const double maxAllowedTransitionCost = std::numeric_limits<double>::max())
{
	std::vector<double> times;
	for (int i = 0; i < static_cast<int>(trellis.size()); ++i) { times.push_back(i); }

	return viterbi(trellis, times, stateCostFn, transitionCostFn, maxAllowedTransitionCost);
}

} // namespace mps

#endif // MPS_VOXELS_VITERBI_H
