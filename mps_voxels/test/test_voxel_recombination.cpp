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

#include "mps_voxels/VoxelRegion.h"

#include <gtest/gtest.h>

#define VISUALIZE false
#if VISUALIZE
#include <ros/ros.h>
#include <mps_voxels/MarkerSet.h>
#include <mps_voxels/visualization/dispersed_colormap.h>
#include <mps_voxels/visualization/visualize_voxel_region.h>
#include <mps_voxels/visualization/visualize_occupancy.h>
#endif


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

	return result;
}

using IndivisibleCluster = std::vector<mps::VoxelRegion::vertex_descriptor>;

namespace mps
{

std::pair<VoxelRegion::VertexLabels, std::map<std::pair<int, int>, double>> computeSegmentationGraph(const VoxelRegion& vox, const std::vector<VoxelRegion::VertexLabels>& particles)
{
	#if VISUALIZE
	static ros::NodeHandle nh;
	static ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("test", 1, true);
	static std::random_device rd;
	static std::default_random_engine re(rd());
	#endif
	// Crop to active region?
	// Compute indivisible clusters
	VoxelRegion::EdgeState definiteEdges(vox.num_edges(), true);
	VoxelRegion::EdgeState possibleEdges(vox.num_edges(), false);
	std::vector<bool> mightBeOccupied(vox.num_vertices(), false);

	for (size_t p = 0; p < particles.size(); ++p)
	{
		for (VoxelRegion::vertices_size_type v = 0; v < vox.num_vertices(); ++v)
		{
			if (particles[p][v] != VoxelRegion::FREE_SPACE)
			{
				mightBeOccupied[v] = true;
			}
		}
	}

//	#pragma omp parallel for
	for (size_t p = 0; p < particles.size(); ++p)
	{
		for (VoxelRegion::edges_size_type e = 0; e < vox.num_edges(); ++e)
		{
			const VoxelRegion::edge_descriptor edge = vox.edge_at(e);
			auto i = vox.index_of(source(edge, vox));
			auto j = vox.index_of(target(edge, vox));

			auto a = particles[p][i];
			auto b = particles[p][j];

			bool isPossibleEdge = mightBeOccupied[i] && mightBeOccupied[j] && (a == b);
//			#pragma omp critical
			{
				definiteEdges[e] = definiteEdges[e] && isPossibleEdge;
				possibleEdges[e] = possibleEdges[e] || isPossibleEdge;
			}
		}
	}
	for (VoxelRegion::edges_size_type e = 0; e < vox.num_edges(); ++e)
	{
		if (definiteEdges[e]) { possibleEdges[e] = false; }
	}
	auto clusters = vox.components(definiteEdges);

	#if VISUALIZE
	std::map<int, std_msgs::ColorRGBA> cmap;
	auto colors = dispersedColormap(clusters.first);
	for (size_t i = 0; i < colors.size(); ++i) { cmap.emplace(static_cast<int>(i)+1, colors[i]); }
	while (ros::ok())
	{
		sleep(1);
		std_msgs::Header header;
		header.frame_id = "table_surface";
		header.stamp = ros::Time::now();
		MarkerSet allMarkers;
		allMarkers["clusters"] = mps::visualize(vox, clusters.second, header, cmap);
		allMarkers["definiteEdges"] = mps::visualize(vox, definiteEdges, header, re);
		allMarkers["possibleEdges"] = mps::visualize(vox, possibleEdges, header, re);
		allMarkers["a"] = mps::visualize(vox, particles[0], header, re);
		allMarkers["b"] = mps::visualize(vox, particles[1], header, re);
		visualPub.publish(allMarkers.flatten());
		assert(possibleEdges[30]);
	}
	#endif

	// Compute graph
	using ClusterEdge = std::pair<int, int>;
	std::map<ClusterEdge, double> weights;
	for (VoxelRegion::edges_size_type e = 0; e < vox.num_edges(); ++e)
	{
		if (possibleEdges[e])
		{
			const VoxelRegion::edge_descriptor edge = vox.edge_at(e);
			auto i = vox.index_of(source(edge, vox));
			auto j = vox.index_of(target(edge, vox));

			auto alpha = clusters.second[i];
			auto beta = clusters.second[j];
			assert(alpha != VoxelRegion::FREE_SPACE);
			assert(beta != VoxelRegion::FREE_SPACE);

			for (size_t p = 0; p < particles.size(); ++p)
			{
				auto a = particles[p][i];
				auto b = particles[p][j];

				bool isEdgeInParticle = mightBeOccupied[i] && mightBeOccupied[j] && (a == b);
				if (isEdgeInParticle)
				{
					weights[{alpha, beta}] = 1.0; // += 1.0;
				}
			}
		}
	}

	for (const auto& w : weights)
	{
		std::cout << w.first.first << " -> " << w.first.second << ": " << w.second << std::endl;
	}

	return {clusters.second, weights};
}

}

TEST(recombination, clustering)
{
	mps::VoxelRegion vox({3, 3, 3}, 0.1, Eigen::Vector3d::Zero(), "table_surface");

	computeSegmentationGraph(vox, generateParticles());

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
