//
// Created by arprice on 1/21/19.
//

#ifndef MPS_VOXELS_MAP_NEAREST_HPP
#define MPS_VOXELS_MAP_NEAREST_HPP

//////////////////////////////////////////////////////////////////////////////
//// Map Nearest-Neighbor Lookup
//////////////////////////////////////////////////////////////////////////////

template <class K, class V>
typename std::map<K,V>::const_iterator find_nearest(const typename std::map<K,V>& m, const K& k)
{
	typename std::map<K,V>::const_iterator upper = m.lower_bound(k);
	if (upper == m.begin() || upper->first == k)
	{
		return upper;
	}
	typename std::map<K,V>::const_iterator lower = upper;
	--lower;

	if (upper == m.end() ||
	    (k-lower->first) < (upper->first-k))
	{
		return lower;
	}

	return upper;
}

#endif //MPS_VOXELS_MAP_NEAREST_HPP
