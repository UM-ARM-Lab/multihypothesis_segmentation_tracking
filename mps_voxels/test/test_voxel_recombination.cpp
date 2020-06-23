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
#include <memory>

#include <gtest/gtest.h>

#define VISUALIZE true
#if VISUALIZE
#include <ros/ros.h>
#include <mps_voxels/MarkerSet.h>
#include <mps_voxels/visualization/dispersed_colormap.h>
#include <mps_voxels/visualization/visualize_voxel_region.h>
#include <mps_voxels/visualization/visualize_occupancy.h>
#endif


using namespace mps;

std::vector<mps::VoxelRegion::VertexLabels> generateParticlesA()
{
	std::vector<mps::VoxelRegion::VertexLabels> result;
	result.push_back({1, 1});
	result.push_back({1, 2});
	result.push_back({1, 1});
	result.push_back({1, 1});
	result.push_back({1, 2});
	result.push_back({1, 2});

	return result;
}

std::vector<mps::VoxelRegion::VertexLabels> generateParticlesB()
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




//
//vertex_component_map mapping = boost::make_shared<mapping_t>();
//size_t num_components = boost::connected_components(G2, boost::associative_property_map<mapping_t>(*mapping));
//if (num_components > 1)
//{
////		throw std::logic_error("Graph is disconnected.");
//}

TEST(recombination, analytics)
{
	#if VISUALIZE
	static ros::NodeHandle nh;
	static ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("test", 1, true);
	#endif
	auto vox = std::make_shared<mps::VoxelRegion>(boost::array<std::size_t, 3>{2, 1, 1}, 0.1, Eigen::Vector3d::Zero(), "table_surface");

	auto P = generateParticlesA();
	std::random_device rd;
	std::mt19937 re(rd());

	for (size_t numToUse = 2; numToUse <= P.size(); ++numToUse)
	{
		std::vector<const VoxelRegion::VertexLabels*> toCombine;
		for (size_t p = 0; p < numToUse; ++p)
		{
			toCombine.emplace_back(&(P[p]));
		}

		VoxelConflictResolver resolver(vox, toCombine);

		resolver.print(std::cerr);

		std::vector<int> histogram(2, 0);
		int nSamples = 1000;
		for (int iter = 0; iter < nSamples; ++iter)
		{
			auto structure = resolver.sampleStructure(re);
			auto V = resolver.sampleGeometry(toCombine, structure, re);
			if (V[0] == V[1])
			{
				++histogram[0];
			}
			else
			{
				++histogram[1];
			}
		}
		std::cerr << "n = " << numToUse << " : " << histogram[0]/(double)nSamples << "\t" << histogram[1]/(double)nSamples << std::endl;

		#if VISUALIZE
		if (numToUse == P.size())
		{
			std::map<int, std_msgs::ColorRGBA> cmap;
			auto colors = dispersedColormap(resolver.vertexLookup.size());
			for (size_t i = 0; i < colors.size(); ++i) { cmap.emplace(static_cast<int>(i) + 1, colors[i]); }
			for (int iter = 0; iter < 100; ++iter)
			{
				auto structure = resolver.sampleStructure(re);
				auto V = resolver.sampleGeometry(toCombine, structure, re);
				sleep(1);
				std_msgs::Header header;
				header.frame_id = "table_surface";
				header.stamp = ros::Time::now();
				MarkerSet allMarkers;
				for (size_t p = 0; p < P.size(); ++p)
				{
					allMarkers["particle_" + std::to_string(p)] = mps::visualize(*vox, P[p], header, re);
				}
				allMarkers["clusters"] = mps::visualize(*vox, V, header, cmap);
				visualPub.publish(allMarkers.flatten());
			}
		}
		#endif
	}
}


TEST(recombination, clustering)
{
	#if VISUALIZE
	static ros::NodeHandle nh;
	static ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("test", 1, true);
	#endif
	auto vox = std::make_shared<mps::VoxelRegion>(boost::array<std::size_t, 3>{3, 3, 3}, 0.1, Eigen::Vector3d::Zero(), "table_surface");

	std::vector<const VoxelRegion::VertexLabels*> toCombine;
	auto P = generateParticlesB();
	for (const auto& p : P) { toCombine.emplace_back(&p);}

	VoxelConflictResolver resolver(vox, toCombine);

	resolver.print(std::cerr);

	std::random_device rd;
	std::mt19937 re(rd());

	#if VISUALIZE
	std::map<int, std_msgs::ColorRGBA> cmap;
	auto colors = dispersedColormap(resolver.vertexLookup.size());
	for (size_t i = 0; i < colors.size(); ++i) { cmap.emplace(static_cast<int>(i)+1, colors[i]); }
	for (int iter = 0; iter < 100; ++iter)
	{
		auto structure = resolver.sampleStructure(re);
		auto V = resolver.sampleGeometry(toCombine, structure, re);
		sleep(1);
		std_msgs::Header header;
		header.frame_id = "table_surface";
		header.stamp = ros::Time::now();
		MarkerSet allMarkers;
		allMarkers["clusters"] = mps::visualize(*vox, V, header, cmap);
		visualPub.publish(allMarkers.flatten());
	}
	#endif

	computeSegmentationGraph(*vox, toCombine);

	std::cerr << "# Vertices: " << vox->num_vertices() << std::endl;
	std::cerr << "# Edges: " << vox->num_edges() << std::endl;

	boost::grid_graph<2> grid(boost::array<std::size_t, 2>{{2, 3}});
	std::cerr << num_vertices(grid) << std::endl;
	std::cerr << get(boost::vertex_index, grid, {{1, 1}});

	for (mps::VoxelRegion::vertices_size_type v = 0; v < vox->num_vertices(); ++v)
	{
		ASSERT_EQ(vox->index_of(vox->vertex_at(v)), v);
	}

	for (mps::VoxelRegion::edges_size_type e = 0; e < vox->num_edges(); ++e)
	{
		ASSERT_EQ(vox->index_of(vox->edge_at(e)), e);
	}


	std::vector<bool> edgeValues(vox->num_edges(), false);

	mps::VoxelRegion::vertex_descriptor a{{0, 0, 0}};
	mps::VoxelRegion::vertex_descriptor b{{0, 0, 1}};
	mps::VoxelRegion::vertex_descriptor c{{0, 1, 1}};
	mps::VoxelRegion::vertex_descriptor d{{0, 1, 0}};

	auto e1 = vox->getEdgeIndex(a, b);
	auto e2 = vox->getEdgeIndex(b, c);
	auto e3 = vox->getEdgeIndex(c, d);
	ASSERT_LT(e1, vox->num_edges());
	ASSERT_LT(e2, vox->num_edges());
	ASSERT_LT(e3, vox->num_edges());
	ASSERT_EQ(vox->getEdgeIndex(c, d), vox->getEdgeIndex(d, c));
	edgeValues[e1] = true;
	edgeValues[e2] = true;
	edgeValues[e3] = true;


//	std::vector<int> vertexValues(num_vertices(grid), 0);


	auto comps = vox->components(edgeValues).second;

	ASSERT_EQ(comps[vox->index_of(a)], comps[vox->index_of(d)]);
	ASSERT_TRUE(edgeValues[vox->getEdgeIndex(a, d)]);
	ASSERT_FALSE(edgeValues[vox->getEdgeIndex(a, {1, 0, 0})]);
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
