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


using SegmentIndex = std::pair<ros::Time, int>;

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

using VideoSegmentationGraph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, NodeProperties, EdgeProperties>;
using SegmentLookup = std::map<SegmentIndex, VideoSegmentationGraph::vertex_descriptor>;

std::deque<SegmentIndex> getObjectPath(const VideoSegmentationGraph& G, const SegmentLookup& segmentToNode, const SegmentIndex& from, const SegmentIndex& to);

#endif // MAP_GRAPH_H
