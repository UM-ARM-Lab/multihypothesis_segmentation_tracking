//
// Created by arprice on 12/14/18.
//

#include "mps_voxels/video_graph.h"

#include <boost/graph/astar_search.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>
#include <boost/property_map/transform_value_property_map.hpp>


template <class Graph, class CostType>
class distance_heuristic : public boost::astar_heuristic<Graph, CostType>
{
public:
	typedef typename boost::graph_traits<Graph>::vertex_descriptor Vertex;
	distance_heuristic(const VideoSegmentationGraph& G_, Vertex goal_)
		: G(G_), m_goal(goal_) {}

	CostType operator()(Vertex u)
	{
		const NodeProperties& currentNode = G[u];
		const NodeProperties& goalNode = G[m_goal];

		return (currentNode.t-goalNode.t).toSec();
	}
private:
	const VideoSegmentationGraph& G;
	Vertex m_goal;
};

struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
	astar_goal_visitor(Vertex goal) : m_goal(goal) {}
	template <class Graph>
	void examine_vertex(Vertex u, Graph&) {
		if(u == m_goal)
			throw found_goal();
	}
private:
	Vertex m_goal;
};


std::deque<SegmentIndex> getObjectPath(const VideoSegmentationGraph& G, const SegmentLookup& segmentToNode, const SegmentIndex& from, const SegmentIndex& to)
{
	// Setup storage for intermediate values in A*
	std::vector<VideoSegmentationGraph::vertex_descriptor> predecessor(boost::num_vertices(G));
	std::vector<double> distance(boost::num_vertices(G));

	// Tell boost about the distance function and temporary storage
	auto wmap = boost::make_transform_value_property_map([](const EdgeProperties& e) { return e.distance(); }, get(boost::edge_bundle, G));
	auto pmap = boost::make_iterator_property_map(predecessor.begin(), boost::get(boost::vertex_index, G));
	auto dmap = boost::make_iterator_property_map(distance.begin(), boost::get(boost::vertex_index, G));

	VideoSegmentationGraph::vertex_descriptor start = segmentToNode.at(from);
	VideoSegmentationGraph::vertex_descriptor goal = segmentToNode.at(to);

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
			                               visitor(astar_goal_visitor<VideoSegmentationGraph::vertex_descriptor>(goal)));
	}
	catch (found_goal fg)
	{
		std::deque<SegmentIndex> res;
		for(VideoSegmentationGraph::vertex_descriptor v = goal;; v = predecessor[v])
		{
			const NodeProperties& np = G[v];
			res.push_front({np.t, np.leafID});
			if(predecessor[v] == v)
				break;
		}
		return res;
	}

	throw std::logic_error("Failed to find path.");
	return {};
}
