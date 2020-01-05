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

#include <boost/array.hpp>
#include <boost/graph/grid_graph.hpp>
#include <boost/graph/connected_components.hpp>

#include <vector>
#include <iostream>

namespace mps
{

class VoxelSegmentation
{
public:
	const static int Dimensions = 3;
	using Grid = boost::grid_graph<Dimensions>;
	using VertexCoordinates = Grid::vertex_descriptor;
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

	size_t getEdgeIndex(VertexCoordinates a, VertexCoordinates b)
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

		// TODO: Flatten and relabel tree as root labels

		return dsf.nodes;
	}

	using edges_size_type = Grid::edges_size_type;
	using edge_descriptor = Grid::edge_descriptor;
	using vertices_size_type = Grid::vertices_size_type;
	using vertex_descriptor = Grid::vertex_descriptor;

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

TEST(segmentation, image_update)
{
	mps::VoxelSegmentation vox({3, 3, 3});
//	std::cerr << "# Vertices: " << num_vertices(vox.grid) << std::endl;
//	std::cerr << "# Edges: " << num_edges(vox.grid) << std::endl;
//
//	std::cerr << get(boost::vertex_index, vox.grid, mps::VoxelSegmentation::VertexCoordinates{0, 0, 0});

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

	mps::VoxelSegmentation::VertexCoordinates a{{0, 0, 0}};
	mps::VoxelSegmentation::VertexCoordinates b{{0, 0, 1}};
	mps::VoxelSegmentation::VertexCoordinates c{{0, 1, 1}};
	mps::VoxelSegmentation::VertexCoordinates d{{0, 1, 0}};

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


int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);

	return RUN_ALL_TESTS();
}