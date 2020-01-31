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

#include "mps_voxels/ValueTree.h"

#include <cassert>

#include <iostream>

namespace mps
{

int root(const ValueTree& T)
{
	assert(!T.parent.empty());

	int node = 0;
	int parent = 0;
	do
	{
		node = parent;
		parent = T.parent[node];
	} while (parent!=node);

	return parent;
}

std::set<int> ancestors(const ValueTree& T, const int node)
{
	std::set<int> res;
	int n = node;
	int parent = node;
	do
	{
		n = parent;
		res.insert(n);
		parent = T.parent[n];
	} while (parent!=n);

	res.erase(node);

	return res;
}

void descendants(const ValueTree& T, const int node, std::set<int>& nodes)
{
	const auto& children = T.children[node];

	nodes.insert(children.begin(), children.end());

	for (const int c : children)
	{
		descendants(T, c, nodes);
	}
}

double value(const ValueTree& T, const TreeCut& C)
{
	double v = 0;
	for (const int c : C)
	{
		v += T.value[c];
	}
	return v;
}

void bottomup_pass(const ValueTree& T, const int node, std::vector<double>& best_child_cut_value)
{
	const auto& children = T.children[node];

	if (children.empty())
	{
		best_child_cut_value[node] = T.value[node];
		return;
	}
	else
	{
		double max_weight = 0;
		for (const int c : children)
		{
			bottomup_pass(T, c, best_child_cut_value);
			max_weight += std::max(T.value[c], best_child_cut_value[c]);
		}
		best_child_cut_value[node] = max_weight;
	}
}

void topdown_pass(const ValueTree& T, const int node, const std::vector<double>& best_child_cut_value, TreeCut& C)
{
	if (T.value[node] >= best_child_cut_value[node])
	{
		// Cutting here is as good or better than any lower cut.
		C.insert(node);
		return;
	}
	else
	{
		// Cutting at a lower level is better. Continue down the tree.
		const auto& children = T.children[node];
		for (const int c : children)
		{
			topdown_pass(T, c, best_child_cut_value, C);
		}
	}
}

std::pair<double, TreeCut> optimalCut(const ValueTree& T)
{
	int r = root(T);
	std::vector<double> best_child_cut_value(T.value.size(), 0.0);
	bottomup_pass(T, r, best_child_cut_value);

	TreeCut C;
	topdown_pass(T, r, best_child_cut_value, C);

	return { value(T, C), C };
}

}
