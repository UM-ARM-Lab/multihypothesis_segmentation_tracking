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

#include "mps_voxels/DisjointSetForest.hpp"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/ProbVoxel.h"
#include "mps_voxels/octree_utils.h"

#include <boost/array.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/graph/connected_components.hpp>

#include <ompl/util/RandomNumbers.h>

#include <octomap/octomap.h>
#include <octomap/OcTreeStamped.h>
#include <Eigen/Geometry>
#include <unordered_set>

#include <vector>
#include <iostream>

namespace mps
{

class VoxelSegmentation
{
public:
	const static int Dimensions = 3;
	using Grid = boost::grid_graph<Dimensions>;
	using vertex_descriptor = Grid::vertex_descriptor;
	// vertex_descriptor is the boost::array of the grid coordinates
	// VertexIndex is the size_t linear position of the vertex
	// edge_descriptor is the ordered pair (vertex_descriptor, vertex_descriptor)

	using EdgeState = std::vector<bool>;
	using VertexLabels = std::vector<int>;

	VoxelSegmentation(boost::array<std::size_t, Dimensions> dims)
		: m_dimension_lengths(dims)
	{
		precalculate();
	}

	size_t getEdgeIndex(vertex_descriptor a, vertex_descriptor b)
	{
		// Normalize the ordering
		auto i = index_of(a);
		auto j = index_of(b);
		if (j < i) { std::swap(a, b); }

		// Check for adjacency (boost::edge(a, b) does this, but return the descriptor and not the ID.)
		int dist = 0;
		for (size_t d = 0; d < dimensions(); ++d)
		{
			dist += abs(static_cast<long>(a[d]) - static_cast<long>(b[d]));
		}
		if (dist != 1) { throw std::logic_error("Grid vertices for edge lookup are not adjacent."); }

		// Return the edge
		return index_of({a, b});
	}

	// From edge graph to vertex labels
	VertexLabels components(EdgeState& edges)
	{
		DisjointSetForest<int> dsf(num_vertices());

		for (Grid::edges_size_type e = 0; e < num_edges(); ++e) // edges are bidirectional :(
		{
			if (edges[e])
			{
				const Grid::edge_descriptor edge = edge_at(e);
				auto i = index_of(source(edge, *this));
				auto j = index_of(target(edge, *this));
				dsf.merge(i, j);
			}
		}

		// Normalize the edges
		for (Grid::edges_size_type e = 0; e < num_edges(); ++e) // edges are bidirectional :(
		{
			const Grid::edge_descriptor edge = edge_at(e);
			auto i = index_of(source(edge, *this));
			auto j = index_of(target(edge, *this));

			edges[e] = (dsf.nodes[i] == dsf.nodes[j]);
		}

		// Flatten and relabel tree as root labels
		for (size_t i = 0; i < dsf.nodes.size(); ++i)
		{
			dsf.nodes[i] = dsf.getAncestor(i);
		}

		return dsf.nodes;
	}

	visualization_msgs::MarkerArray visualizeEdgeState(EdgeState& edges, const double resolution,
		const Eigen::Vector3d& roiMin, const std::string& globalFrame, const std_msgs::ColorRGBA* base_color)
	{
		VertexLabels vlabels = components(edges);
		Eigen::Vector3d offset(resolution * 0.5, resolution * 0.5, resolution * 0.5);
		octomap::OcTree octree(resolution);
		for (size_t i = 0; i < vlabels.size(); i++)
		{
			const vertex_descriptor vd = vertex_at(i);
			Eigen::Vector3d coord = roiMin + resolution * (Eigen::Map<const Eigen::Matrix<std::size_t, 3, 1>>(vd.data()).cast<double>()) + offset;
			octree.updateNode(coord[0], coord[1], coord[2], true);
		}
		return visualizeOctree(&octree, globalFrame, base_color);
	}

	visualization_msgs::MarkerArray visualizeEdgeStateDirectly(EdgeState& edges, const double resolution,
	                                                   const Eigen::Vector3d& roiMin, const std::string& globalFrame)
	{
		visualization_msgs::MarkerArray edgeStateVis;
		edgeStateVis.markers.resize(1);
		VertexLabels vlabels = components(edges);
		Eigen::Vector3d offset(resolution * 0.5, resolution * 0.5, resolution * 0.5);
		for (size_t i = 0; i < vlabels.size(); i++)
		{
			const vertex_descriptor vd = vertex_at(i);
			Eigen::Vector3d coord = roiMin + resolution * (Eigen::Map<const Eigen::Matrix<std::size_t, 3, 1>>(vd.data()).cast<double>()) + offset;

			geometry_msgs::Point cubeCenter;
			cubeCenter.x = coord[0];
			cubeCenter.y = coord[1];
			cubeCenter.z = coord[2];

			edgeStateVis.markers[0].points.push_back(cubeCenter);

			// Colors
			std_msgs::ColorRGBA color;
			color.a = 1.0;
			color.r = 0.5;
			color.g = 0.0;
			color.b = 1.0;
			edgeStateVis.markers[0].colors.push_back(color);

		}

		edgeStateVis.markers[0].header.frame_id = globalFrame;
		edgeStateVis.markers[0].header.stamp = ros::Time::now();
		edgeStateVis.markers[0].ns = "map" + std::to_string(0);//"occlusion";
		edgeStateVis.markers[0].id = 0;
		edgeStateVis.markers[0].type = visualization_msgs::Marker::CUBE_LIST;
		edgeStateVis.markers[0].scale.x = resolution;
		edgeStateVis.markers[0].scale.y = resolution;
		edgeStateVis.markers[0].scale.z = resolution;
		edgeStateVis.markers[0].color.r = 0;
		edgeStateVis.markers[0].color.g = 0.2;
		edgeStateVis.markers[0].color.b = 1;
		edgeStateVis.markers[0].color.a = 1;

		if (edgeStateVis.markers[0].points.size()>0)
			edgeStateVis.markers[0].action = visualization_msgs::Marker::ADD;
		else
			edgeStateVis.markers[0].action = visualization_msgs::Marker::DELETE;

		return edgeStateVis;
	}


	using edges_size_type = Grid::edges_size_type;
	using edge_descriptor = Grid::edge_descriptor;
	using vertices_size_type = Grid::vertices_size_type;

	const vertex_descriptor m_dimension_lengths;
	vertices_size_type m_num_vertices;

	boost::array<edges_size_type, Dimensions> m_edge_count;
	edges_size_type m_num_edges;

	// Pre-computes the number of vertices and edges
	void precalculate() {
		m_num_vertices =
			std::accumulate(m_dimension_lengths.begin(),
			                m_dimension_lengths.end(),
			                vertices_size_type(1),
			                std::multiplies<>());

		// Calculate number of edges in each dimension
		m_num_edges = 0;

		for (std::size_t dimension_index = 0;
		     dimension_index < Dimensions;
		     ++dimension_index) {

			m_edge_count[dimension_index] =
				(num_vertices() - (num_vertices() / length(dimension_index)));

			m_num_edges += num_edges(dimension_index);
		}
	}

	// Returns the number of dimensions in the graph
	inline std::size_t dimensions() const {
		return (Dimensions);
	}

	// Returns the length of dimension [dimension_index]
	inline vertices_size_type length(std::size_t dimension) const {
		return (m_dimension_lengths[dimension]);
	}

	// Returns the number of vertices in the graph
	inline vertices_size_type num_vertices() const {
		return (m_num_vertices);
	}

	// Returns the number of edges in the graph
	inline edges_size_type num_edges() const {
		return (m_num_edges);
	}

	// Returns the number of edges in dimension [dimension_index]
	inline edges_size_type num_edges(std::size_t dimension_index) const {
		return (m_edge_count[dimension_index]);
	}

	// Gets the vertex that is [distance] units ahead of [vertex] in
	// dimension [dimension_index].
	vertex_descriptor next
		(vertex_descriptor vertex,
		 std::size_t dimension_index,
		 vertices_size_type distance = 1) const {

		vertices_size_type new_position =
			vertex[dimension_index] + distance;

		// Stop at the end of this dimension if necessary.
		new_position =
			(std::min)(new_position,
			           vertices_size_type(length(dimension_index) - 1));

		vertex[dimension_index] = new_position;

		return (vertex);
	}

	// Gets the vertex that is [distance] units behind [vertex] in
	// dimension [dimension_index].
	vertex_descriptor previous
		(vertex_descriptor vertex,
		 std::size_t dimension_index,
		 vertices_size_type distance = 1) const {

		// We're assuming that vertices_size_type is unsigned, so we
		// need to be careful about the math.
		vertex[dimension_index] =
			(distance > vertex[dimension_index]) ? 0 : vertex[dimension_index] - distance;

		return (vertex);
	}

	// Returns the index of [vertex] (See also vertex_at)
	vertices_size_type index_of(vertex_descriptor vertex) const {

		vertices_size_type vertex_index = 0;
		vertices_size_type index_multiplier = 1;

		for (std::size_t dimension_index = 0;
		     dimension_index < Dimensions;
		     ++dimension_index) {

			vertex_index += (vertex[dimension_index] * index_multiplier);
			index_multiplier *= length(dimension_index);
		}

		return (vertex_index);
	}

	// Returns the vertex whose index is [vertex_index] (See also
	// index_of(vertex_descriptor))
	vertex_descriptor vertex_at(vertices_size_type vertex_index) const {

		boost::array<vertices_size_type, Dimensions> vertex;
		vertices_size_type index_divider = 1;

		for (std::size_t dimension_index = 0;
		     dimension_index < Dimensions;
		     ++dimension_index) {

			vertex[dimension_index] = (vertex_index / index_divider) %
			                          length(dimension_index);

			index_divider *= length(dimension_index);
		}

		return (vertex);
	}

	// Returns the edge whose index is [edge_index] (See also
	// index_of(edge_descriptor)).
	edge_descriptor edge_at(edges_size_type edge_index) const {

		// Edge indices are sorted into bins by dimension
		std::size_t dimension_index = 0;
		edges_size_type dimension_edges = num_edges(0);

		while (edge_index >= dimension_edges) {
			edge_index -= dimension_edges;
			++dimension_index;
			dimension_edges = num_edges(dimension_index);
		}

		vertex_descriptor vertex_source, vertex_target;

		// Dimensions can wrap arbitrarily, so an index needs to be
		// computed in a more complex manner.  This is done by
		// grouping the edges for each dimension together into "bins"
		// and considering [edge_index] as an offset into the bin.

		edges_size_type vertex_offset = edge_index % num_edges(dimension_index);

		// Consider vertex_offset an index into the graph's vertex
		// space but with the dimension [dimension_index] reduced in
		// size by one.
		vertices_size_type index_divider = 1;

		for (std::size_t dimension_index_iter = 0;
		     dimension_index_iter < Dimensions;
		     ++dimension_index_iter) {

			std::size_t dimension_length = (dimension_index_iter == dimension_index) ?
			                               length(dimension_index_iter) - 1 :
			                               length(dimension_index_iter);

			vertex_source[dimension_index_iter] = (vertex_offset / index_divider) %
			                                      dimension_length;

			index_divider *= dimension_length;
		}

		vertex_target = next(vertex_source, dimension_index);

		return (std::make_pair(vertex_source, vertex_target));
	}

	edges_size_type index_of(edge_descriptor edge) const {
		vertex_descriptor source_vertex = source(edge, *this);
		vertex_descriptor target_vertex = target(edge, *this);

		BOOST_ASSERT (source_vertex != target_vertex);

		// Determine the dimension where the source and target vertices
		// differ (should only be one if this is a valid edge).
		std::size_t different_dimension_index = 0;

		while (source_vertex[different_dimension_index] ==
		       target_vertex[different_dimension_index]) {

			++different_dimension_index;
		}

		edges_size_type edge_index = 0;

		// Offset the edge index into the appropriate "bin" (see edge_at
		// for a more in-depth description).
		for (std::size_t dimension_index = 0;
		     dimension_index < different_dimension_index;
		     ++dimension_index) {

			edge_index += num_edges(dimension_index);
		}

		// Finally, apply the vertex offset
		vertices_size_type index_multiplier = 1;

		for (std::size_t dimension_index = 0;
		     dimension_index < Dimensions;
		     ++dimension_index) {

			edge_index += (source_vertex[dimension_index] * index_multiplier);
			index_multiplier *= (dimension_index == different_dimension_index) ?
			                    length(dimension_index) - 1 :
			                    length(dimension_index);
		}


		return (edge_index);
	}

};

}

//#include "mps_voxels/VoxelSegmentation.h"


#include <gtest/gtest.h>

using namespace boost; // Boo! But currently needed for num_vertices...

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

template <typename Point>
Point snap(const Point& p, const octomap::OcTree* octree)
{
	auto coord = octree->keyToCoord(octree->coordToKey(p.x(), p.y(), p.z()));
	return {coord.x(), coord.y(), coord.z()};
}

mps::VoxelSegmentation::vertex_descriptor roiToGrid(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& roiMax)
{
	auto minCoord = snap(roiMin, octree);
	auto maxCoord = snap(roiMax, octree);

	mps::VoxelSegmentation::vertex_descriptor dims;
	for (int i = 0; i < 3; ++i)
	{
		dims[i] = ceil((maxCoord[i]-minCoord[i])/octree->getResolution()) + 1;
	}

	return dims;
}

mps::VoxelSegmentation::vertex_descriptor coordToGrid(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& query)
{
	auto minCoord = snap(roiMin, octree);
	auto queryCoord = snap(query, octree);

	mps::VoxelSegmentation::vertex_descriptor dims;
	for (int i = 0; i < 3; ++i)
	{
		dims[i] = (queryCoord[i]-minCoord[i])/octree->getResolution();
	}

	return dims;
}

Eigen::Vector3d gridToCoord(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const mps::VoxelSegmentation::vertex_descriptor& query)
{
	Eigen::Vector3d offset(octree->getResolution() * 0.5, octree->getResolution() * 0.5, octree->getResolution() * 0.5);
	return roiMin + octree->getResolution() * (Eigen::Map<const Eigen::Matrix<std::size_t, 3, 1>>(query.data()).cast<double>()) + offset;
}

bool isOccupied(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const mps::VoxelSegmentation::vertex_descriptor& query)
{
	auto coord = gridToCoord(octree, roiMin, query);
	octomap::OcTreeNode* node = octree->search(coord.x(), coord.y(), coord.z());
	if (node)
	{
		return node->getOccupancy() > 0.5;
	}
	return false;
}

std::pair<bool, double>
sampleIsOccupied(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const mps::VoxelSegmentation::vertex_descriptor& query,
                 ompl::RNG& rng)
{
	auto coord = gridToCoord(octree, roiMin, query);
	octomap::OcTreeNode* node = octree->search(coord.x(), coord.y(), coord.z());

	bool a;
	double pa;
	if (node)
	{
		a = node->getOccupancy() > rng.uniform01();
		pa = a ? node->getOccupancy() : (1.0-node->getOccupancy());
	}
	else
	{
		a = false;
		pa = 1.0;
	}
	return {a, pa};
}

// From octree labels to edge graph
mps::VoxelSegmentation::EdgeState
octreeToGrid(const octomap::OcTree* octree,
             const Eigen::Vector3d& minExtent,
             const Eigen::Vector3d& maxExtent)
{
	mps::VoxelSegmentation::vertex_descriptor dims = roiToGrid(octree, minExtent, maxExtent);
	mps::VoxelSegmentation vox(dims);

	mps::VoxelSegmentation::EdgeState edges(vox.num_edges());

	for (mps::VoxelSegmentation::edges_size_type e = 0; e < vox.num_edges(); ++e)
	{
		const mps::VoxelSegmentation::edge_descriptor edge = vox.edge_at(e);
		auto i = source(edge, vox);
		auto j = target(edge, vox);

		bool a = isOccupied(octree, minExtent, i);
		bool b = isOccupied(octree, minExtent, j);

		edges[e] = (a && b);
	}

	return edges;
}

// From octree labels to edge graph
std::pair<double, mps::VoxelSegmentation::EdgeState>
octreeToGridParticle(const octomap::OcTree* octree,
                     const Eigen::Vector3d& minExtent,
                     const Eigen::Vector3d& maxExtent,
                     ompl::RNG& rng)
{
	mps::VoxelSegmentation::vertex_descriptor dims = roiToGrid(octree, minExtent, maxExtent);
	mps::VoxelSegmentation vox(dims);

	mps::VoxelSegmentation::EdgeState edges(vox.num_edges());

	double logOdds = 0;
	for (mps::VoxelSegmentation::edges_size_type e = 0; e < vox.num_edges(); ++e)
	{
		const mps::VoxelSegmentation::edge_descriptor edge = vox.edge_at(e);
		auto i = source(edge, vox);
		auto j = target(edge, vox);

		bool a; // Whether a cell is active
		bool b;
		double pa; // Probability of that cell state
		double pb;

		std::tie(a, pa) = sampleIsOccupied(octree, minExtent, i, rng);
		std::tie(b, pb) = sampleIsOccupied(octree, minExtent, j, rng);

		edges[e] = (a && b);
		logOdds += std::log(pa) + std::log(pb);
	}

	return {logOdds, edges};
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

TEST(segmentation, octreelabel) {
	octomap::OcTreeStamped ots(0.05);
//	octomap::OcTreeLabel otl(0.05);

}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}
