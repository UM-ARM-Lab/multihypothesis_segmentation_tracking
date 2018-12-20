/**
 * \file map_graph.h
 * \brief
 *
 * \author Andrew Price
 * \date 2016-8-1
 *
 * \copyright
 *
 * Copyright (c) 2016, Georgia Tech Research Corporation
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

#ifndef MAP_GRAPH_H
#define MAP_GRAPH_H

#include <boost/graph/adjacency_list.hpp>
#include <ros/time.h>


//using SegmentIndex = std::pair<ros::Time, int>;

enum class SEGMENT_TYPE
{
	UCM,
	BODY
};

template <SEGMENT_TYPE T>
struct SegmentIndex
{
	ros::Time first = ros::Time(0);
	long second = -1;

//	SegmentIndex() : first(0.0), second(-1) {}
//	SegmentIndex(const std::pair<ros::Time, int>& p) : first(p.first), second(p.second) {}

	bool operator<(const SegmentIndex& other) const
	{
		if (first < other.first)
		{
			return true;
		}
		else if (first > other.first)
		{
			return false;
		}
		else
		{
			return second < other.second;
		}
	}
};


template <SEGMENT_TYPE T>
struct NodeProperties
{
	ros::Time t; ///< Time index in video
	int leafID; ///< Segment index in frame-wise UCM
	int component; ///< Assigned Component

	NodeProperties() : t(0.0), leafID(-1), component(-1) {}
	NodeProperties(const std::pair<ros::Time, int>& p) : t(p.first), leafID(p.second), component(-1) {}
};

struct EdgeProperties
{
	double affinity = 0.0;
	double distance() const { return 1.0/affinity; }
};

// Helper types for using for_each
template <typename Iterator>
class iterator_pair
{
public:
	iterator_pair ( Iterator first, Iterator last ) : f_ (first), l_ (last) {}
	iterator_pair ( std::pair<Iterator, Iterator> p ) : f_ (p.first), l_ (p.second) {}
	Iterator begin () const { return f_; }
	Iterator end   () const { return l_; }

private:
	Iterator f_;
	Iterator l_;
};
template <typename Iterator>
iterator_pair<Iterator> make_range ( std::pair<Iterator, Iterator> p )
{
	return iterator_pair<Iterator> ( p );
}

template <SEGMENT_TYPE T>
using VideoSegmentationGraph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, NodeProperties<T>, EdgeProperties>;

template <SEGMENT_TYPE T>
using SegmentLookup = std::map<SegmentIndex<T>, typename VideoSegmentationGraph<T>::vertex_descriptor>;

template <SEGMENT_TYPE T>
std::deque<SegmentIndex<T>> getObjectPath(const VideoSegmentationGraph<T>& G, const SegmentLookup<T>& segmentToNode, const SegmentIndex<T>& from, const SegmentIndex<T>& to);

#endif // MAP_GRAPH_H
