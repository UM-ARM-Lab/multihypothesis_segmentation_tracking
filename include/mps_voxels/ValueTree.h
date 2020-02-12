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

#ifndef SRC_VALUETREE_H
#define SRC_VALUETREE_H

#include <vector>
#include <set>
#include <map>

namespace mps
{

namespace tree
{

using NodeID = int;
using TreeCut = std::set<NodeID>;

struct DenseValueTree
{
	std::vector<NodeID> parent_;
	std::vector<std::vector<NodeID>> children_;
	std::vector<double> value_;
};

struct SparseValueTree
{
	std::map<NodeID, NodeID> parent_;
	std::map<NodeID, std::vector<NodeID>> children_;
	std::map<NodeID, double> value_;
};

template <typename ValueTree>
size_t size(const ValueTree& T);

template <typename ValueTree>
const NodeID& parent(const ValueTree& T, const NodeID node);

template <typename ValueTree>
NodeID& parent(ValueTree& T, const NodeID node);

template <typename ValueTree>
const std::vector<NodeID>& children(const ValueTree& T, const NodeID node);

template <typename ValueTree>
std::vector<NodeID>& children(ValueTree& T, const NodeID node);

template <typename ValueTree>
double value(const ValueTree& T, const NodeID node);

template <typename ValueTree>
double& value(ValueTree& T, const NodeID node);

template <typename ValueTree>
NodeID first(const ValueTree& T);

template <typename ValueTree>
NodeID root(const ValueTree& T);

template <typename ValueTree>
std::set<NodeID> ancestors(const ValueTree& T, const NodeID node);

template <typename ValueTree>
void descendants(const ValueTree& T, const NodeID node, std::set<NodeID>& nodes);

template <typename ValueTree>
double value(const ValueTree& T, const TreeCut& C);

/**
 *
 * @param T
 * @param node
 * @param best_child_cut_value The maximum value of a cut existing below this node
 */
template <typename ValueTree>
void bottomup_pass(const ValueTree& T, const NodeID node, std::vector<double>& best_child_cut_value);

template <typename ValueTree>
void topdown_pass(const ValueTree& T, const NodeID node, const std::vector<double>& best_child_cut_value, TreeCut& C);

template <typename ValueTree>
std::pair<double, TreeCut>
optimalCut(const ValueTree& T);

}

}

#endif // SRC_VALUETREE_H
