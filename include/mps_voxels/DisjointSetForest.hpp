//
// Created by arprice on 9/7/19.
//

#ifndef SRC_DISJOINTSETFOREST_HPP
#define SRC_DISJOINTSETFOREST_HPP

#include <vector>
#include <numeric>

template <typename Index = unsigned>
class DisjointSetForest
{
public:
	using LUT = std::vector<Index>;
	LUT nodes;

	explicit
	DisjointSetForest(size_t num_elements)
		: nodes(num_elements)
	{
		std::iota(nodes.begin(), nodes.end(), 0);
	}

	Index getAncestor(const Index& i)
	{
		Index parent = nodes[i];

		if (i == parent)
		{
			// Self-referential element, root
			return i;
		}
		else
		{
			const Index& ancestor = getAncestor(parent);
			nodes[i] = ancestor;
			return ancestor;
		}
	}

	Index merge(const Index& i, const Index& j)
	{
		//assert(i != j); // Merge i, i promotes i to the set representative
		Index ancestorI = getAncestor(i);
		Index ancestorJ = getAncestor(j);
		nodes[j] = ancestorI;
		nodes[ancestorJ] = ancestorI;
		return ancestorI;
	}

};


#endif // SRC_DISJOINTSETFOREST_HPP
