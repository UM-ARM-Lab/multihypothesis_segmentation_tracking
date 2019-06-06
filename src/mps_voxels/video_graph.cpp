//
// Created by arprice on 12/14/18.
//

#include "mps_voxels/video_graph.h"
#include "mps_voxels/assert.h"

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