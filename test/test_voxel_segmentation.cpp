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

#include "mps_voxels/VoxelSegmentation.h"

#include <octomap/octomap.h>
#include <Eigen/Geometry>
#include <unordered_set>


#include <gtest/gtest.h>

using namespace mps;

TEST(segmentation, clustering)
{
	mps::VoxelSegmentation vox({3, 3, 3});
//	std::cerr << "# Vertices: " << num_vertices(vox.grid) << std::endl;
//	std::cerr << "# Edges: " << num_edges(vox.grid) << std::endl;
//
//	std::cerr << get(boost::vertex_index, vox.grid, mps::VoxelSegmentation::vertex_descriptor{0, 0, 0});

//	boost::grid_graph<2> grid(boost::array<std::size_t, 2>{{2, 3}});
//	std::cerr << num_vertices(grid) << std::endl;
//	std::cerr << get(boost::vertex_index, grid, {{1, 1}});

	for (mps::VoxelSegmentation::vertices_size_type v = 0; v < vox.num_vertices(); ++v)
	{
		ASSERT_EQ(vox.index_of(vox.vertex_at(v)), v);
	}

	for (mps::VoxelSegmentation::edges_size_type e = 0; e < vox.num_edges(); ++e)
	{
		ASSERT_EQ(vox.index_of(vox.edge_at(e)), e);
	}


	std::vector<bool> edgeValues(vox.num_edges(), false);

	mps::VoxelSegmentation::vertex_descriptor a{{0, 0, 0}};
	mps::VoxelSegmentation::vertex_descriptor b{{0, 0, 1}};
	mps::VoxelSegmentation::vertex_descriptor c{{0, 1, 1}};
	mps::VoxelSegmentation::vertex_descriptor d{{0, 1, 0}};

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


	auto comps = vox.components(edgeValues);

	ASSERT_EQ(comps[vox.index_of(a)], comps[vox.index_of(d)]);
	ASSERT_TRUE(edgeValues[vox.getEdgeIndex(a, d)]);
	ASSERT_FALSE(edgeValues[vox.getEdgeIndex(a, {1, 0, 0})]);
	std::cerr << "Yay!" << std::endl;

}


#pragma clang diagnostic push
#pragma ide diagnostic ignored "cert-flp30-c"
TEST(segmentation, octree)
{
	std::shared_ptr<octomap::OcTree> octree = std::make_shared<octomap::OcTree>(0.05);
	Eigen::Vector3d min(0.0, -0.2, 0.0);
	Eigen::Vector3d max(0.1, -0.0, 0.2);
	octomath::Vector3 octmin(min[0], min[1], min[2]);
	octomath::Vector3 octmax(max[0], max[1], max[2]);
	octree->setBBXMin(octmin);
	octree->setBBXMax(octmax);

	// insert some measurements of occupied cells
	std::cerr << int((max[0] - min[0])/octree->getResolution()) << std::endl;
	std::cerr << int((max[1] - min[1])/octree->getResolution()) << std::endl;
	std::cerr << int((max[2] - min[2])/octree->getResolution()) << std::endl;

	std::set<octomap::OcTreeNode*> nodes;
	const float eps = std::numeric_limits<float>::epsilon();
	const float res = octree->getResolution() + eps;
	int count  = 0;
	for (float x = min[0]; x <= max[0]; x += res)
	{
		for (float y = min[1]; y <= max[1]; y += res)
		{
			for (float z = min[2]; z <= max[2]; z += res)
			{
				auto* node = octree->updateNode(x, y, z, true);
				++count;
				ASSERT_TRUE(nodes.insert(node).second);
			}
		}
	}


	if (octree->getRoot()){
		std::cerr << "OcTree is not empty!" << std::endl;
	}
	std::cerr << "Number of nodes = " << octree->calcNumNodes() << std::endl;

	std::cerr << "count = " << count << std::endl;

	Eigen::Vector3d minLoop = gridToCoord(octree.get(), min, coordToGrid(octree.get(), min, min));
	ASSERT_TRUE((minLoop-min).isZero(octree->getResolution()));

	Eigen::Vector3d maxLoop = gridToCoord(octree.get(), min, coordToGrid(octree.get(), min, max));
	ASSERT_TRUE((maxLoop-max).isZero(octree->getResolution()));


	mps::VoxelSegmentation::vertex_descriptor dims = roiToGrid(octree.get(), min, max);
	mps::VoxelSegmentation vox(dims);

	// Verify 1-1 correspondance between octree leafs and grid nodes
	std::unordered_set<octomap::OcTreeKey, octomap::OcTreeKey::KeyHash> keys;

	std::cerr << "resolution_factor = " << 1.0/octree->getResolution() << std::endl;
	for (mps::VoxelSegmentation::vertices_size_type v = 0; v < vox.num_vertices(); ++v)
	{
		Eigen::Vector3d p = gridToCoord(octree.get(), min, vox.vertex_at(v));
		std::cerr << "p = " << p[0] << " " << p[1] << " " << p[2] << std::endl;
		std::cerr << "my key = " << (int) floor(1.0/octree->getResolution() * p[0]) << " " << (int) floor(1.0/octree->getResolution() * p[1]) << " " << (int) floor(1.0/octree->getResolution() * p[2]) << std::endl;

		octomap::OcTreeKey key = octree->coordToKey(p.x(), p.y(), p.z());
		std::cerr << "key = " << key[0] << " " << key[1] << " " << key[2] << std::endl;
		ASSERT_TRUE(keys.find(key) == keys.end()); // key is unique
		keys.insert(key);
	}

	// TODO: Verify that every Octree leaf has a unique grid node

	Eigen::Vector3d p(0.01, -0.22, 0.11);
	auto coord = snap(p, octree.get());
	std::cerr << coord[0] << " " << coord[1] << " " << coord[2] << std::endl;

	for (auto node = octree->begin_leafs(); node != octree->end_leafs(); node++){
		std::cerr << "Node center: " << node.getCoordinate();
		std::cerr << " value: " << node->getValue() << "\n";
	}

	// TODO: Investigate what happens when you call index_of(edge_descriptor) without having the vertices ordered correctly
	mps::VoxelSegmentation::vertex_descriptor v1, v2;
	v1 = vox.vertex_at(1);
	v2 = vox.vertex_at(2);

	mps::VoxelSegmentation::edge_descriptor e1, e2;
	e1.first = v1;
	e1.second = v2;
	e2.first = v2;
	e2.second = v1;
	if (vox.index_of(e1) == vox.index_of(e2)){
		std::cerr << "Commutative!" << std::endl;
	}
	else{
		std::cerr << "V1 -> V2: " << vox.index_of(e1) << std::endl;
		std::cerr << "V2 -> V1: " << vox.index_of(e2) << std::endl;
	}

	// TODO: Instantiate a simple OcTree, convert it to a grid graph, convert it back, and visualize it

//	{
//		visualization_msgs::MarkerArray ma = visualizeOctree(octree, globalFrame, &mapColor);
//		for (visualization_msgs::Marker& m : ma.markers)
//		{
//			m.ns = "map";
//		}
//		allMarkers["map"] = ma;
//		octreePub.publish(allMarkers.flatten());
//	}

}
#pragma clang diagnostic pop

/* TODO:
 * OcTree is originally used to store occupancy probability, while we don't care about occupancy probability.
 * Changing float to int -> store label? or use ColorOcTree.
 * Some reference:
 * https://answers.ros.org/question/11443/initializing-and-manipulating-octomaps-with-more-than-just-occupancy/ */

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}