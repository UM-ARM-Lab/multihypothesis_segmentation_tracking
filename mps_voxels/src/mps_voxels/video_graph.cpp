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

#include "mps_voxels/video_graph.h"
#include "mps_voxels/util/assert.h"

#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/transform_value_property_map.hpp>

#include <deque>


struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class goal_visitor : public boost::default_dijkstra_visitor
{
public:
	explicit
	goal_visitor(Vertex goal) : m_goal(goal) {}

	template <class Graph>
	void examine_vertex(Vertex u, Graph&)
	{
		if(u == m_goal)
			throw found_goal();
	}
private:
	Vertex m_goal;
};


template <SEGMENT_TYPE T>
std::deque<SegmentIndex<T>> getObjectPath(const VideoSegmentationGraph<T>& G, const SegmentLookup<T>& segmentToNode, const SegmentIndex<T>& from, const SegmentIndex<T>& to)
{
	using Vertex = typename VideoSegmentationGraph<T>::vertex_descriptor;
	// Setup storage for intermediate values in A*
	std::vector<Vertex> predecessor(boost::num_vertices(G));
	std::vector<double> distance(boost::num_vertices(G));

	// Tell boost about the distance function and temporary storage
	auto wmap = boost::make_transform_value_property_map([](const EdgeProperties& e) { return e.distance(); }, get(boost::edge_bundle, G));
	auto pmap = boost::make_iterator_property_map(predecessor.begin(), boost::get(boost::vertex_index, G));
	auto dmap = boost::make_iterator_property_map(distance.begin(), boost::get(boost::vertex_index, G));

	Vertex start = segmentToNode.at(from);
	Vertex goal = segmentToNode.at(to);

	MPS_ASSERT(start < boost::num_vertices(G));
	MPS_ASSERT(goal < boost::num_vertices(G));

//	try
//	{
//		boost::astar_search(G, start, distance_heuristic<VideoSegmentationGraph, double>(G, goal),
//		    boost::weight_map(wmap).predecessor_map(pmap).distance_map(dmap).
//		    visitor(astar_goal_visitor<VideoSegmentationGraph::vertex_descriptor>(segmentToNode.at(to))));
//	}
//	catch (found_goal fg)
//	{
//		std::deque<SegmentIndex> res;
//		for(VideoSegmentationGraph::vertex_descriptor v = goal;; v = predecessor[v])
//		{
//			const NodeProperties& np = G[v];
//			res.push_front({np.t, np.leafID});
//			if(predecessor[v] == v)
//				break;
//		}
//		return res;
//	}

	try
	{
		boost::dijkstra_shortest_paths(G, start,
		                               boost::weight_map(wmap).predecessor_map(pmap).distance_map(dmap).
			                               visitor(goal_visitor<Vertex>(goal)));
	}
	catch (found_goal fg)
	{
		std::deque<SegmentIndex<T>> res;
		for(Vertex v = goal;; v = predecessor[v])
		{
			const NodeProperties<T>& np = G[v];
			res.push_front({np.t, np.leafID});
			if(predecessor[v] == v)
				break;
		}
		return res;
	}

//	throw std::logic_error("Failed to find path.");
	return {};
}

template
std::deque<SegmentIndex<SEGMENT_TYPE::UCM>> getObjectPath(const VideoSegmentationGraph<SEGMENT_TYPE::UCM>& G, const SegmentLookup<SEGMENT_TYPE::UCM>& segmentToNode, const SegmentIndex<SEGMENT_TYPE::UCM>& from, const SegmentIndex<SEGMENT_TYPE::UCM>& to);

template
std::deque<SegmentIndex<SEGMENT_TYPE::BODY>> getObjectPath(const VideoSegmentationGraph<SEGMENT_TYPE::BODY>& G, const SegmentLookup<SEGMENT_TYPE::BODY>& segmentToNode, const SegmentIndex<SEGMENT_TYPE::BODY>& from, const SegmentIndex<SEGMENT_TYPE::BODY>& to);