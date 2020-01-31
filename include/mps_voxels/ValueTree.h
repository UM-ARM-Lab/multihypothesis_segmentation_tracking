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

namespace mps
{

struct ValueTree
{
	std::vector<int> parent;
	std::vector<std::vector<int>> children;
	std::vector<double> value;
};

using TreeCut = std::set<int>;

int root(const ValueTree& T);

std::set<int>
ancestors(const ValueTree& T, const int node);

void descendants(const ValueTree& T, const int node, std::set<int>& nodes);

double value(const ValueTree& T, const TreeCut& C);

/**
 *
 * @param T
 * @param node
 * @param best_child_cut_value The maximum value of a cut existing below this node
 */
void bottomup_pass(const ValueTree& T, const int node, std::vector<double>& best_child_cut_value);

void topdown_pass(const ValueTree& T, const int node, const std::vector<double>& best_child_cut_value, TreeCut& C);

std::pair<double, TreeCut>
optimalCut(const ValueTree& T);

}

#endif // SRC_VALUETREE_H
