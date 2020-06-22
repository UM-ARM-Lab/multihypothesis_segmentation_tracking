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

#define VISUALIZE true
#if VISUALIZE
#include <ros/ros.h>
#endif

#include "mps_voxels/VoxelRegion.h"
#include "mps_voxels/voxel_recombination.h"

#include <gtest/gtest.h>

#include "mps_voxels/DisjointSetForest.hpp"

#include <boost/bimap.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/random_spanning_tree.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/range/iterator_range.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <random>

#define VISUALIZE true
#if VISUALIZE
#include <ros/ros.h>
#include <mps_voxels/MarkerSet.h>
#include <mps_voxels/visualization/dispersed_colormap.h>
#include <mps_voxels/visualization/visualize_voxel_region.h>
#include <mps_voxels/visualization/visualize_occupancy.h>
#endif

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


using namespace mps;

std::vector<mps::VoxelRegion::VertexLabels> generateParticles()
{
	std::vector<mps::VoxelRegion::VertexLabels> result;
	result.push_back({1, 1, 0,
	                  0, 0, 0,
	                  0, 0, 2, // ---
	                  1, 1, 0,
	                  0, 0, 0,
	                  0, 0, 2, // ---
	                  1, 1, 0,
	                  0, 0, 0,
	                  0, 0, 2});
	result.push_back({1, 1, 0,
	                  0, 0, 0,
	                  0, 0, 2, // ---
	                  1, 1, 0,
	                  0, 0, 0,
	                  0, 0, 2, // ---
	                  3, 3, 0,
	                  3, 0, 0,
	                  3, 0, 2});
	result.push_back({1, 1, 0,
	                  0, 0, 0,
	                  0, 0, 2, // ---
	                  1, 1, 0,
	                  0, 0, 0,
	                  0, 0, 2, // ---
	                  3, 3, 3,
	                  3, 0, 0,
	                  0, 0, 2});

	return result;
}

using ObjectInstance = std::pair<ParticleIndex, ObjectIndex>;

struct ConflictEdgeProperties
{
	int numOverlaps = 0;
};

using ConflictGraph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, ObjectInstance, ConflictEdgeProperties>;
using NodeLookup = boost::bimap<ObjectInstance, ConflictGraph::vertex_descriptor>;
using component_mapping_t = std::map<ConflictGraph::vertex_descriptor, unsigned long>;
using vertex_component_map = std::shared_ptr<component_mapping_t>;
using ComponentOrdering = std::vector<std::map<ConflictGraph::vertex_descriptor, ConflictGraph::vertex_descriptor>>;

//struct Visitor : public boost::dfs_visitor<>
//{
//	template <class Vertex, class Graph>
//	void discover_vertex(const Vertex v, const Graph& g)
//	{
////		std::cerr <<
//
//};

using ComponentGraph = boost::filtered_graph<ConflictGraph, std::function<bool(ConflictGraph::edge_descriptor)>, std::function<bool(ConflictGraph::vertex_descriptor)>>;

class VoxelConflictResolver
{
public:
	ConflictGraph G;
	NodeLookup vertexLookup;
	vertex_component_map components;
	std::vector<std::vector<ConflictGraph::vertex_descriptor>> componentToMembers;
	std::vector<ComponentGraph> componentGraphs;
	std::map<ObjectInstance, int> objectSizes;

	boost::shared_mutex graphMtx;
	boost::shared_mutex lookupMtx;

	VoxelConflictResolver(const VoxelRegion& vox, const std::vector<const VoxelRegion::VertexLabels*>& particles)
	{
		for (size_t v = 0; v < vox.num_vertices(); ++v)
		{
			std::vector<ObjectInstance> objectsAtThisCell;
			for (size_t p = 0; p < particles.size(); ++p)
			{
				int label = (*particles[p])[v];
				if (label != VoxelRegion::FREE_SPACE)
				{
					ObjectInstance inst{ParticleIndex(p), ObjectIndex(label)};
					objectsAtThisCell.push_back(inst);
					objectSizes[inst]++;
				}
			}

			for (size_t i = 0; i < objectsAtThisCell.size(); ++i)
			{
				ConflictGraph::vertex_descriptor m = getOrAddVertex(objectsAtThisCell[i], G, vertexLookup);
				for (size_t j = i+1; j < objectsAtThisCell.size(); ++j)
				{
					ConflictGraph::vertex_descriptor n = getOrAddVertex(objectsAtThisCell[j], G, vertexLookup);

					boost::unique_lock<boost::shared_mutex> graphLock(graphMtx);
					auto res = boost::add_edge(m, n, G);
					ConflictEdgeProperties& eProps = G[res.first];
					++eProps.numOverlaps;
				}
			}
		}

		components = std::make_shared<component_mapping_t>();
		size_t num_components = boost::connected_components(G, boost::associative_property_map<component_mapping_t>(*components));

		componentToMembers.resize(num_components);
		for (size_t i = 0; i < components->size(); ++i) { componentToMembers[components->at(i)].push_back(i); }

		for (size_t i = 0; i < num_components; i++)
		{
			componentGraphs.emplace_back(G,
			                             [this, i](ConflictGraph::edge_descriptor e)
			                              {
				                              return this->components->at(source(e, this->G)) == i
				                                     || this->components->at(target(e, this->G)) == i;
			                              },
			                             [this, i](ConflictGraph::vertex_descriptor v)
			                              {
				                              return this->components->at(v) == i;
			                              });
		}

		for (size_t i = 0; i < num_components; i++)
		{
			const auto& cg = componentGraphs[i];
			std::cerr << "graph G {\n";
			for (ConflictGraph::edge_descriptor ed : make_range(boost::edges(cg)))
			{
				ConflictGraph::vertex_descriptor u = boost::source(ed, cg);
				ConflictGraph::vertex_descriptor v = boost::target(ed, cg);

				const ConflictEdgeProperties& ep = cg[ed];
				const ObjectInstance& npu = vertexLookup.right.at(u);
				const ObjectInstance& npv = vertexLookup.right.at(v);
				std::cerr << node_name(npu) << "--" << node_name(npv) << " [penwidth=\"" << ep.numOverlaps << "\"];\n";
			}
			std::cerr << "}\n";
		}



	}

	ConflictGraph::vertex_descriptor getOrAddVertex(const ObjectInstance& inst, ConflictGraph& g, NodeLookup& lookup)
	{
		boost::upgrade_lock<boost::shared_mutex> lock(lookupMtx);
		auto iter = lookup.left.find(inst);
		if (iter == lookup.left.end())
		{
			boost::upgrade_to_unique_lock< boost::shared_mutex > uniqueLock(lock);
			boost::unique_lock<boost::shared_mutex> graphLock(graphMtx);
			auto res = boost::add_vertex(g);
			lookup.insert({inst, res});
			return res;
		}
		return iter->second;
	}

	inline std::string node_name(const ObjectInstance& obj) const { return "n" + std::to_string(obj.first.id) + "_" + std::to_string(obj.second.id); }

	void print(std::ostream& out) const
	{
		out << "graph G {\n";

//	std::map<ros::Time, std::vector<ConflictGraph::vertex_descriptor>> frames;
//	for (ConflictGraph::vertex_descriptor vd : make_range(boost::vertices(G)))
//	{
//		frames[G[vd].t].push_back(vd);
//	}
//
//	for (const auto& frame : frames)
//	{
//		out << "subgraph cluster_" << frame.first << " {\n";
//		out << "label=\"Frame " << frame.first << "\";\n";
//		for (const ConflictGraph::vertex_descriptor vd : frame.second)
//		{
//			const NodeProperties& np = G[vd];
//			out << node_name(np) << " [label=\"" << np.leafID << "\"];\n";
//		}
//		out << "}\n";
//	}

		for (ConflictGraph::edge_descriptor ed : make_range(boost::edges(G)))
		{
			ConflictGraph::vertex_descriptor u = boost::source(ed, G);
			ConflictGraph::vertex_descriptor v = boost::target(ed, G);

			const ConflictEdgeProperties& ep = G[ed];
			const ObjectInstance& npu = vertexLookup.right.at(u);
			const ObjectInstance& npv = vertexLookup.right.at(v);
			out << node_name(npu) << "--" << node_name(npv) << " [penwidth=\"" << ep.numOverlaps << "\"];\n";
		}

		out << "}\n";
	}

	ComponentOrdering
	sampleStructure(std::default_random_engine& re) const
	{
		std::vector<std::map<ConflictGraph::vertex_descriptor, ConflictGraph::vertex_descriptor>> structures;
		for (size_t i = 0; i < componentGraphs.size(); i++)
		{
			const auto& cg = componentGraphs[i];

			// Pick the root uniformly
			std::uniform_int_distribution<> dis(0, componentToMembers[i].size() - 1);
			const ConflictGraph::vertex_descriptor root = componentToMembers[i].at(dis(re));

//			std::vector<int> weightBar; objectSizes
//			std::discrete_distribution<int> distribution(weightBar.begin(), weightBar.end());

			std::map<ConflictGraph::vertex_descriptor, ConflictGraph::vertex_descriptor> predecessors;
			boost::associative_property_map<std::map<ConflictGraph::vertex_descriptor, ConflictGraph::vertex_descriptor>> pmap(predecessors);
			boost::random_spanning_tree(cg, re, boost::predecessor_map(pmap).root_vertex(root));

			structures.push_back(predecessors);

			std::cerr << "digraph G {\n";
			for (const auto& pair : predecessors)
			{
				if (pair.second == static_cast<ConflictGraph::vertex_descriptor>(-1)) { continue; }
				const ObjectInstance& npu = vertexLookup.right.at(pair.first);
				const ObjectInstance& npv = vertexLookup.right.at(pair.second);
				ConflictGraph::edge_descriptor ed = boost::edge(pair.first, pair.second, G).first;
				const ConflictEdgeProperties& ep = cg[ed];
				std::cerr << node_name(npu) << "->" << node_name(npv) << " [penwidth=\"" << ep.numOverlaps << "\"];\n";
			}
			std::cerr << "}\n";
		}

		return structures;
	}

	VoxelRegion::VertexLabels
	sampleGeometry(const VoxelRegion& vox, const std::vector<const VoxelRegion::VertexLabels*>& particles, const ComponentOrdering& structures, std::default_random_engine& re) const
	{
		std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));

		DisjointSetForest<int> dsf(vertexLookup.size());

		using Digraph = boost::adjacency_list<boost::setS, boost::vecS, boost::directedS>;
		Digraph digraph;
		std::vector<Digraph::vertex_descriptor> revOrdering;

		for (size_t component = 0; component < structures.size(); ++component)
		{
			const auto& predecessors = structures[component];


			for (const auto& pair : predecessors)
			{
				if (pair.second == static_cast<ConflictGraph::vertex_descriptor>(-1)) { continue; }

				boost::add_edge(pair.second, pair.first, digraph);
			}

			// Attempt 1: Independent edge sampling
			for (const auto& pair : predecessors)
			{
				if (pair.second == static_cast<ConflictGraph::vertex_descriptor>(-1)) { continue; }

				const ObjectInstance& npu = vertexLookup.right.at(pair.first);
				const ObjectInstance& npv = vertexLookup.right.at(pair.second);
				ConflictGraph::edge_descriptor ed = boost::edge(pair.first, pair.second, G).first;
				const ConflictEdgeProperties& ep = G[ed];

				double iou = static_cast<double>(ep.numOverlaps)/static_cast<double>(objectSizes.at(npu) + objectSizes.at(npv) - ep.numOverlaps);
				bool doMerge = uni(re) < iou;
				if (doMerge)
				{
					dsf.merge(pair.second, pair.first);
				}
			}
		}

		boost::topological_sort(digraph, std::back_inserter(revOrdering));

		std::cerr << "digraph G {\n";
		for (Digraph::edge_descriptor ed : make_range(boost::edges(digraph)))
		{
			ConflictGraph::vertex_descriptor u = boost::source(ed, digraph);
			ConflictGraph::vertex_descriptor v = boost::target(ed, digraph);

			const ObjectInstance& npu = vertexLookup.right.at(u);
			const ObjectInstance& npv = vertexLookup.right.at(v);
			std::cerr << node_name(npu) << "->" << node_name(npv) << ";\n";
		}
		std::cerr << "}\n";

		VoxelRegion::VertexLabels result(vox.num_vertices(), VoxelRegion::FREE_SPACE);

		for (const auto vd : revOrdering)
		{
			const ObjectInstance& npu = vertexLookup.right.at(vd);
			for (size_t v = 0; v < vox.num_vertices(); ++v)
			{
				int plabel = (*particles[npu.first.id])[v];
				if (plabel == npu.second.id)
				{
					int id = dsf.getAncestor(vd);
					result[v] = id + 1;
				}
			}
		}

		// TODO: What about when we cut up a node?

		// Normalize the vertex state
		VoxelRegion::EdgeState edges(vox.num_edges(), false);
		for (VoxelRegion::edges_size_type e = 0; e < vox.num_edges(); ++e)
		{
			const VoxelRegion::edge_descriptor edge = vox.edge_at(e);
			auto i = vox.index_of(source(edge, vox));
			auto j = vox.index_of(target(edge, vox));

			auto a = result[i];
			auto b = result[j];

			edges[e] = (a != VoxelRegion::FREE_SPACE) && (a == b);
		}

		result = vox.components(edges).second;

		return result;
	}
};



//
//vertex_component_map mapping = boost::make_shared<mapping_t>();
//size_t num_components = boost::connected_components(G2, boost::associative_property_map<mapping_t>(*mapping));
//if (num_components > 1)
//{
////		throw std::logic_error("Graph is disconnected.");
//}



TEST(recombination, clustering)
{
	#if VISUALIZE
	static ros::NodeHandle nh;
	static ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("test", 1, true);
	#endif
	mps::VoxelRegion vox({3, 3, 3}, 0.1, Eigen::Vector3d::Zero(), "table_surface");

	std::vector<const VoxelRegion::VertexLabels*> toCombine;
	auto P = generateParticles();
	for (const auto& p : P) { toCombine.emplace_back(&p);}

	VoxelConflictResolver resolver(vox, toCombine);

	resolver.print(std::cerr);

	std::random_device rd;
	std::default_random_engine re(rd());

	#if VISUALIZE
	std::map<int, std_msgs::ColorRGBA> cmap;
	auto colors = dispersedColormap(resolver.vertexLookup.size());
	for (size_t i = 0; i < colors.size(); ++i) { cmap.emplace(static_cast<int>(i)+1, colors[i]); }
	for (int iter = 0; iter < 100; ++iter)
	{
		auto structure = resolver.sampleStructure(re);
		auto V = resolver.sampleGeometry(vox, toCombine, structure, re);
		sleep(1);
		std_msgs::Header header;
		header.frame_id = "table_surface";
		header.stamp = ros::Time::now();
		MarkerSet allMarkers;
		allMarkers["clusters"] = mps::visualize(vox, V, header, cmap);
		visualPub.publish(allMarkers.flatten());
	}
	#endif

	computeSegmentationGraph(vox, toCombine);

	std::cerr << "# Vertices: " << vox.num_vertices() << std::endl;
	std::cerr << "# Edges: " << vox.num_edges() << std::endl;

	boost::grid_graph<2> grid(boost::array<std::size_t, 2>{{2, 3}});
	std::cerr << num_vertices(grid) << std::endl;
	std::cerr << get(boost::vertex_index, grid, {{1, 1}});

	for (mps::VoxelRegion::vertices_size_type v = 0; v < vox.num_vertices(); ++v)
	{
		ASSERT_EQ(vox.index_of(vox.vertex_at(v)), v);
	}

	for (mps::VoxelRegion::edges_size_type e = 0; e < vox.num_edges(); ++e)
	{
		ASSERT_EQ(vox.index_of(vox.edge_at(e)), e);
	}


	std::vector<bool> edgeValues(vox.num_edges(), false);

	mps::VoxelRegion::vertex_descriptor a{{0, 0, 0}};
	mps::VoxelRegion::vertex_descriptor b{{0, 0, 1}};
	mps::VoxelRegion::vertex_descriptor c{{0, 1, 1}};
	mps::VoxelRegion::vertex_descriptor d{{0, 1, 0}};

	auto e1 = vox.getEdgeIndex(a, b);
	auto e2 = vox.getEdgeIndex(b, c);
	auto e3 = vox.getEdgeIndex(c, d);
	ASSERT_LT(e1, vox.num_edges());
	ASSERT_LT(e2, vox.num_edges());
	ASSERT_LT(e3, vox.num_edges());
	ASSERT_EQ(vox.getEdgeIndex(c, d), vox.getEdgeIndex(d, c));
	edgeValues[e1] = true;
	edgeValues[e2] = true;
	edgeValues[e3] = true;


//	std::vector<int> vertexValues(num_vertices(grid), 0);


	auto comps = vox.components(edgeValues).second;

	ASSERT_EQ(comps[vox.index_of(a)], comps[vox.index_of(d)]);
	ASSERT_TRUE(edgeValues[vox.getEdgeIndex(a, d)]);
	ASSERT_FALSE(edgeValues[vox.getEdgeIndex(a, {1, 0, 0})]);
	std::cerr << "Yay!" << std::endl;

}


int main(int argc, char **argv)
{
	#if VISUALIZE
	ros::init(argc, argv, "test_voxel_recombination");
	#endif
	::testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
