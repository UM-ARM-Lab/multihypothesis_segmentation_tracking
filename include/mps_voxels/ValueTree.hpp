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

#ifndef SRC_VALUETREE_HPP
#define SRC_VALUETREE_HPP

#include "mps_voxels/ValueTree.h"

namespace mps
{

namespace tree
{

template<> inline
size_t size<DenseValueTree>(const DenseValueTree& T)
{
	return T.value_.size();
}

template<> inline
size_t size<SparseValueTree>(const SparseValueTree& T)
{
	return T.value_.size();
}

template<> inline
const NodeID& parent<DenseValueTree>(const DenseValueTree& T, const NodeID node)
{
	return T.parent_[node];
}

template<> inline
NodeID& parent<DenseValueTree>(DenseValueTree& T, const NodeID node)
{
	return T.parent_[node];
}

template<> inline
const NodeID& parent<SparseValueTree>(const SparseValueTree& T, const NodeID node)
{
	return T.parent_.at(node);
}

template<> inline
NodeID& parent<SparseValueTree>(SparseValueTree& T, const NodeID node)
{
	return T.parent_.at(node);
}

template<> inline
const std::vector<NodeID>& children<DenseValueTree>(const DenseValueTree& T, const NodeID node)
{
	return T.children_[node];
}

template<> inline
std::vector<NodeID>& children<DenseValueTree>(DenseValueTree& T, const NodeID node)
{
	return T.children_[node];
}

template<> inline
const std::vector<NodeID>& children<SparseValueTree>(const SparseValueTree& T, const NodeID node)
{
	return T.children_.at(node);
}

template<> inline
std::vector<NodeID>& children<SparseValueTree>(SparseValueTree& T, const NodeID node)
{
	return T.children_.at(node);
}

template<> inline
double value<DenseValueTree>(const DenseValueTree& T, const NodeID node)
{
	return T.value_[node];
}

template<> inline
double& value<DenseValueTree>(DenseValueTree& T, const NodeID node)
{
	return T.value_[node];
}

template<> inline
double value<SparseValueTree>(const SparseValueTree& T, const NodeID node)
{
	return T.value_.at(node);
}

template<> inline
double& value<SparseValueTree>(SparseValueTree& T, const NodeID node)
{
	return T.value_.at(node);
}

template <> inline
NodeID first<DenseValueTree>(const DenseValueTree&)
{
	return 0;
}

template <> inline
NodeID first<SparseValueTree>(const SparseValueTree& T)
{
	return T.value_.begin()->first;
}

template <typename ValueTree>
NodeID root(const ValueTree& T)
{
	NodeID node;// = first(T);
	NodeID p = first(T);
	do
	{
		node = p;
		p = parent(T, node);
	}
	while (p != node);

	return p;
}

template <typename ValueTree>
std::set<NodeID> ancestors(const ValueTree& T, const NodeID node)
{
	std::set<NodeID> res;
	NodeID n = node;
	NodeID p = node;
	do
	{
		n = p;
		res.insert(n);
		p = parent(T, n);
	}
	while (p != n);

	res.erase(node);

	return res;
}

template <typename ValueTree>
void descendants(const ValueTree& T, const NodeID node, std::set<NodeID>& nodes)
{
	const auto& C = children(T, node);

	nodes.insert(C.begin(), C.end());

	for (const NodeID c : C)
	{
		descendants(T, c, nodes);
	}
}

template <typename ValueTree>
double value(const ValueTree& T, const TreeCut& C)
{
	double v = 0;
	for (const NodeID c : C)
	{
		v += value(T, c);
	}
	return v;
}

template <typename ValueTree>
struct TreeProperties;

template <>
struct TreeProperties<DenseValueTree>
{
//	using ValueMap = std::vector<double>;
	using ValueMap = std::map<NodeID, double>;
};

template <>
struct TreeProperties<SparseValueTree>
{
	using ValueMap = std::map<NodeID, double>;
};


template <typename ValueTree>
void bottomup_pass(const ValueTree& T, const int node, typename TreeProperties<ValueTree>::ValueMap& best_child_cut_value)
{
	const auto& C = children(T, node);

	if (C.empty())
	{
		best_child_cut_value[node] = value(T, node);
		return;
	}
	else
	{
		double max_weight = 0;
		for (const int c : C)
		{
			bottomup_pass(T, c, best_child_cut_value);
			max_weight += std::max(value(T, c), best_child_cut_value[c]);
		}
		best_child_cut_value[node] = max_weight;
	}
}

template <typename ValueTree>
void topdown_pass(const ValueTree& T, const int node, const typename TreeProperties<ValueTree>::ValueMap& best_child_cut_value, TreeCut& cut)
{
	if (value(T, node) >= best_child_cut_value.at(node))
	{
		// Cutting here is as good or better than any lower cut.
		cut.insert(node);
		return;
	}
	else
	{
		// Cutting at a lower level is better. Continue down the tree.
		const auto& C = children(T, node);
		for (const int c : C)
		{
			topdown_pass(T, c, best_child_cut_value, cut);
		}
	}
}

template <typename ValueTree>
std::pair<double, TreeCut> optimalCut(const ValueTree& T)
{
	NodeID r = root(T);
	using ValueMap = typename TreeProperties<ValueTree>::ValueMap;
	ValueMap best_child_cut_value;//(size(T), 0.0);
	bottomup_pass(T, r, best_child_cut_value);

	TreeCut C;
	topdown_pass(T, r, best_child_cut_value, C);

	return { value(T, C), C };
}

}

}

#endif //SRC_VALUETREE_HPP
