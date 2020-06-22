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

#include <boost/graph/adjacency_list_io.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graphviz.hpp>


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
using ConflictGraph = std::map<ObjectInstance, std::set<ObjectInstance>>;

struct ConflictEdgeProperties
{
	int numOverlaps = 0;
};

using BConflictGraph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, ObjectInstance, ConflictEdgeProperties>;

ConflictGraph
computeConflictGraph(const VoxelRegion& vox, const std::vector<const VoxelRegion::VertexLabels*>& particles)
{
	ConflictGraph cg;

	#pragma omp parallel for
	for (size_t v = 0; v < vox.num_vertices(); ++v)
	{
		std::set<ObjectInstance> objectsAtThisCell;
		for (size_t p = 0; p < particles.size(); ++p)
		{
			int label = (*particles[p])[v];
			if (label != VoxelRegion::FREE_SPACE)
			{
				objectsAtThisCell.emplace(ParticleIndex(p), ObjectIndex(label));
			}
		}

		for (const auto& instance : objectsAtThisCell)
		{
			auto objs = objectsAtThisCell;
			objs.erase(instance);
			#pragma omp critical
			{
				cg[instance].insert(objs.begin(), objs.end());
			}
		}
	}

	return cg;
}

//typedef std::map<VideoSegmentationGraph<SEGMENT_TYPE::BODY>::vertex_descriptor, unsigned long> mapping_t;
//typedef boost::shared_ptr<mapping_t> vertex_component_map;
//
//vertex_component_map mapping = boost::make_shared<mapping_t>();
//size_t num_components = boost::connected_components(G2, boost::associative_property_map<mapping_t>(*mapping));
//if (num_components > 1)
//{
////		throw std::logic_error("Graph is disconnected.");
//}

inline std::string graphName(const ObjectInstance& obj) { return "n" + std::to_string(obj.first.id) + "_" + std::to_string(obj.second.id); }


TEST(recombination, clustering)
{
	mps::VoxelRegion vox({3, 3, 3}, 0.1, Eigen::Vector3d::Zero(), "table_surface");

	std::vector<const VoxelRegion::VertexLabels*> toCombine;
	auto P = generateParticles();
	for (const auto& p : P) { toCombine.emplace_back(&p);}

	auto cg = computeConflictGraph(vox, toCombine);
	std::cerr << "digraph g {" << std::endl;

	for (const auto& entry : cg)
	{
		for (const auto& conflict : entry.second)
		{
			std::cerr << graphName(entry.first) << "->" << graphName(conflict) << ";" << std::endl;
		}
	}
	std::cerr << "}" << std::endl;

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
