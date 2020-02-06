/**
 * \file hungarian.hpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-4-19
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

#ifndef HUNGARIAN_HPP
#define HUNGARIAN_HPP

#include <Eigen/Core>
#include <vector>

// Implementation of the Hungarian Algorithm derived from https://www.topcoder.com/community/data-science/data-science-tutorials/assignment-problem-and-hungarian-algorithm/

namespace mps
{

// TODO: template for multiple cost matrix types

/**
 * @brief The Hungarian class solves the optimal nxn assignment problem in O(n^3) time.
 * The results are reported as a mapping from x to y (row to column of cost matrix)
 *
 * Formally, the system is defined as follows:
 * Given two sets \f$ X, Y \in \mathbb{R}^n \f$ and a cost matrix \f$ C:X \times Y \rightarrow \mathbb{R} \f$,
 * find a bijection \f$ f:X \rightarrow Y \f$ that minimizes the cost function \f$ \sum_{x\in X}C[x,f(x)] \f$.
 */
class Hungarian
{
public:
	Hungarian(const Eigen::MatrixXd& cost);

	/**
	 * @brief getSolutionCost returns the current linear cost of the assignment
	 * @return Solution cost of the form C(x1,xToY(x1)) + C(x2,xToY(x2)) + ...
	 */
	double getSolutionCost();

	/**
	 * @brief getAssignment Returns the optimal choice of y for each x in sequence
	 * @return y-index of optimal pairing for each x
	 */
	const std::vector<int>& getAssignment() { return xy; }

protected:
	Eigen::MatrixXd C;            ///< Cost matrix (nxn)
	const int N;                  ///< Number of pairings to find
	int maxMatch;                 ///< Number of pairings locked so far
	std::vector<double> lx;
	std::vector<double> ly;
	std::vector<int> xy;          ///< Mapping from x to y index
	std::vector<int> yx;          ///< Mapping from y to x index
	std::vector<double> slack;
	std::vector<double> slackx;
	std::vector<int> prev;        ///< Alternating tree
	std::vector<bool> S;          ///< Set of matched x indices
	std::vector<bool> T;          ///< Set of matched y indices

	void augment();
	void relabel();
	void addToTree(int x, int prevx);

};

double Hungarian::getSolutionCost()
{
	double cost = 0;
	for (int x = 0; x < N; ++x)
	{
		cost += C(x, xy[x]);
	}
	return -cost;
}

Hungarian::Hungarian(const Eigen::MatrixXd& cost)
    : C(-cost), // Negate cost, as this algorithm finds the maximum assignment
      N(C.cols()),
      maxMatch(0),
      lx(N, 0),
      ly(N, 0),
      xy(N,-1),
      yx(N,-1),
      slack(N),
      slackx(N),
      prev(N),
      S(N),
      T(N)
{
	// Initial labelling
	for (int x = 0; x < N; ++x)
		for (int y = 0; y < N; ++y)
			lx[x] = std::max(lx[x], C(x,y));

	// Solve
	augment();
}

void Hungarian::relabel()
{
	double delta = std::numeric_limits<double>::infinity();

	for (int y = 0; y < N; ++y) // compute delta with slack
		if (!T[y]) { delta = std::min(delta, slack[y]); }
	assert(std::isfinite(delta));
	for (int x = 0; x < N; ++x) // update X labels
		if (S[x]) { lx[x] -= delta; }
	for (int y = 0; y < N; ++y) // update Y labels
		if (T[y]) { ly[y] += delta; }
	for (int y = 0; y < N; ++y) // update slack array
		if (!T[y]) { slack[y] -= delta; }
}

void Hungarian::addToTree(int x, int prevx)
{
	S[x] = true;
	prev[x] = prevx;
	for (int y = 0; y < N; ++y)
	{
		double newCost = lx[x] + ly[y] - C(x,y);
		if (newCost < slack[y])
		{
			slack[y] = newCost;
			slackx[y] = x;
		}
	}
}

void Hungarian::augment()
{
	if (maxMatch == N) { return; }
	int x = -1, y = -1, root = -1;
	std::vector<int> q(N);
	int wr = 0, rd = 0;

	// Initialize sets and tree
	std::fill(S.begin(), S.end(), false);
	std::fill(T.begin(), T.end(), false);
	std::fill(prev.begin(), prev.end(), -1);

	// Find root of tree
	for (x = 0; x < N; ++x)
	{
		if (-1 == xy[x])
		{
			q[wr++] = root = x;
			prev[x] = -2;
			S[x] = true;
			break;
		}
	}

	// Initialize slack array
	for (y = 0; y < N; ++y)
	{
		slack[y] = lx[root] + ly[y] - C(root,y);
		slackx[y] = root;
	}

	while (true)
	{
		while (rd < wr) // build tree with bfs
		{
			x = q[rd++];
			for (y = 0; y < N; ++y)
			{
				if (C(x,y) == lx[x] + ly[y] && !T[y])
				{
					if (-1 == yx[y]) { break; } // found an isolated Y vertex
					T[y] = true; // else y is in T
					q[wr++] = yx[y]; // add target of y to frontier of bfs
					addToTree(yx[y], x);
				}
			}
			if (y < N) { break; } // Breaking out from inmost loop
		}
		if (y < N) { break; } // Breaking out from inmost loop

		relabel();
		wr = rd = 0;

		for (y = 0; y < N; ++y)
		{
			if (!T[y] && 0 == slack[y])
			{
				if (-1 == yx[y])
				{
					x = slackx[y];
					break;
				}
				else
				{
					T[y] = true;
					if (!S[yx[y]])
					{
						q[wr++] = yx[y];
						addToTree(yx[y], slackx[y]);
					}
				}
			}
		}
		if (y < N) { break; }
	}

	if (y < N)
	{
		maxMatch++;
		for (int cx = x, cy = y, ty; cx != -2; cx = prev[cx], cy = ty)
		{
			ty = xy[cx];
			yx[cy] = cx;
			xy[cx] = cy;
		}
		augment();
	}
}


/*
void hungarian(Eigen::MatrixXd& C)
{
	assert(C.rows() == C.cols());
	const size_t N = C.cols();

	// Initialize
	hungarianInit(C);

}

void hungarianInit(Eigen::MatrixXd& C)
{
	// Subtract smallest element from each row
	for (size_t i = 0; i < N; ++i)
	{
		double rowMin = C(i,0);
		for (size_t j = 0; j < N; ++j)
			if (C(i,j) < rowMin) { rowMin = C(i,j); }
		for (size_t j = 0; j < N; ++j)
			C(i,j) -= rowMin;
	}
	// Subtract smallest element from each column
	for (size_t j = 0; j < N; ++j)
	{
		double colMin = C(0,j);
		for (size_t i = 0; i < N; ++i)
			if (C(i,j) < colMin) { colMin = C(i,j); }
		for (size_t i = 0; i < N; ++i)
			C(i,j) -= colMin;
	}
}
*/
} // namespace gtri
#endif // HUNGARIAN_HPP
