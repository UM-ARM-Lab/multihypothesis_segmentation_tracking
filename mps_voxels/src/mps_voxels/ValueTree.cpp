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
#include "mps_voxels/ValueTree_impl.hpp"

namespace mps
{

namespace tree
{

void compressTree(SparseValueTree& T)
{
	for (auto it = T.children_.cbegin(); it != T.children_.cend(); /* no increment */)
	{
		if (it->second.size() == 1)
		{
			const NodeID& n = it->first;
			const NodeID& c = it->second[0];
			const NodeID& p = parent(T, n);

			value(T, c) = std::max(value(T, n), value(T, c));
			if (n == p)
			{
				// We are the root
				parent(T, c) = c;
			}
			else
			{
				// Promote the only child
				parent(T, c) = p;
				auto& C = children(T, p);
				std::replace(C.begin(), C.end(), n, c);
			}

			// Suicide
			T.parent_.erase(n);
			T.value_.erase(n);
			it = T.children_.erase(it);
		}
		else
		{
			++it;
		}
	}
}

std::pair<DenseValueTree, boost::bimap<NodeID, NodeID>> densify(const SparseValueTree& S)
{
	boost::bimap<NodeID, NodeID> sparseIDtoDenseID;

	int count = 0;
	for (const auto& p : S.value_)
	{
		sparseIDtoDenseID.insert(sparseIDtoDenseID.end(), {p.first, count++});
	}

	DenseValueTree D;
	size_t n = size(S);
	D.value_.resize(n);
	D.parent_.resize(n);
	D.children_.resize(n);
	for (const auto& pair : S.value_)
	{
		const NodeID& s = pair.first; // sparse node
		const NodeID& d = sparseIDtoDenseID.left.at(s); // dense node
		value(D, d) = pair.second;
		parent(D, d) = sparseIDtoDenseID.left.at(parent(S, s));
		children(D, d).reserve(children(S, s).size());
		for (const auto& c : children(S, s))
		{
			children(D, d).push_back(sparseIDtoDenseID.left.at(c));
		}
	}

	return {D, sparseIDtoDenseID};
}

}

}
