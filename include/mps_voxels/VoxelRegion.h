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

#ifndef SRC_VOXELREGION_H
#define SRC_VOXELREGION_H

#include "mps_voxels/Indexes.h"
#include "mps_voxels/Object.h"
//#include "mps_voxels/octree_utils.h"

#include <boost/array.hpp>
#include <boost/graph/grid_graph.hpp>

#include <octomap/octomap.h>

#include <Eigen/Core>
#include <visualization_msgs/MarkerArray.h>

namespace cv
{
class RNG;
}

namespace mps
{

class VoxelRegion
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

	VoxelRegion(boost::array<std::size_t, Dimensions> dims, double res, Eigen::Vector3d rmin, Eigen::Vector3d rmax);

	size_t getEdgeIndex(vertex_descriptor a, vertex_descriptor b);

	// From edge graph to vertex labels
	VertexLabels components(EdgeState& edges) const;

	using edges_size_type = Grid::edges_size_type;
	using edge_descriptor = Grid::edge_descriptor;
	using vertices_size_type = Grid::vertices_size_type;

	double resolution;
	Eigen::Vector3d regionMin;
	Eigen::Vector3d regionMax;
	Eigen::Vector3d regionMinSnapped;

	inline
	bool isInRegion(Eigen::Vector3d query) const
	{
		return query.x() >= regionMin.x() && query.y() >= regionMin.y() && query.z() >= regionMin.z()
		&& query.x() <= regionMax.x() && query.y() <= regionMax.y() && query.z() <= regionMax.z();
	}

	const vertex_descriptor m_dimension_lengths;
	vertices_size_type m_num_vertices;

	boost::array<edges_size_type, Dimensions> m_edge_count;
	edges_size_type m_num_edges;

	// Pre-computes the number of vertices and edges
	void precalculate();

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
		 vertices_size_type distance = 1) const;

	// Gets the vertex that is [distance] units behind [vertex] in
	// dimension [dimension_index].
	vertex_descriptor previous
		(vertex_descriptor vertex,
		 std::size_t dimension_index,
		 vertices_size_type distance = 1) const;

	// Returns the index of [vertex] (See also vertex_at)
	vertices_size_type index_of(vertex_descriptor vertex) const;

	// Returns the vertex whose index is [vertex_index] (See also
	// index_of(vertex_descriptor))
	vertex_descriptor vertex_at(vertices_size_type vertex_index) const;

	// Returns the edge whose index is [edge_index] (See also
	// index_of(edge_descriptor)).
	edge_descriptor edge_at(edges_size_type edge_index) const;

	edges_size_type index_of(edge_descriptor edge) const;

	inline
	Eigen::Vector3d coordinate_of(vertex_descriptor vd) const {
		return regionMinSnapped + resolution * (Eigen::Map<const Eigen::Matrix<std::size_t, 3, 1>>(vd.data()).cast<double>());
	}

	std::map<ObjectIndex, std::shared_ptr<octomap::OcTree>> vertexLabelToOctrees(const VertexLabels& vlabels, const std::set<ObjectIndex>& uniqueObjectLabels);

	VertexLabels objectsToSubRegionVoxelLabel(const std::map<ObjectIndex, std::unique_ptr<Object>>& objects, // object in subRegion
	                                          const Eigen::Vector3d& subRegionMinExtent);
};

template <typename Point>
Point snap(const Point& p, const octomap::OcTree* octree)
{
	auto coord = octree->keyToCoord(octree->coordToKey(p.x(), p.y(), p.z()));
	return {coord.x(), coord.y(), coord.z()};
}

inline
double snapCoord(const double& resolution, const double& x)
{
	return (double( (int) floor( x / resolution) ) + 0.5) * resolution;
}

mps::VoxelRegion::vertex_descriptor roiToGrid(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& roiMax);

mps::VoxelRegion::vertex_descriptor roiToVoxelRegion(const double& resolution, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& roiMax);

mps::VoxelRegion::vertex_descriptor coordToGrid(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& query);

mps::VoxelRegion::vertex_descriptor coordToVertexDesc(const double& resolution, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& query);

Eigen::Vector3d gridToCoord(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const mps::VoxelRegion::vertex_descriptor& query);

Eigen::Vector3d vertexDescpToCoord(const double& resolution, const Eigen::Vector3d& roiMin, const mps::VoxelRegion::vertex_descriptor& query);

bool isOccupied(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const mps::VoxelRegion::vertex_descriptor& query);

std::set<ObjectIndex> getUniqueObjectLabels(const VoxelRegion::VertexLabels& input);

std::pair<bool, double>
sampleIsOccupied(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const mps::VoxelRegion::vertex_descriptor& query,
                 cv::RNG& rng);

/// From octree labels to edge graph
mps::VoxelRegion::EdgeState
octreeToGrid(const octomap::OcTree* octree,
             const Eigen::Vector3d& minExtent,
             const Eigen::Vector3d& maxExtent);

/// From octree labels to edge graph
std::pair<double, mps::VoxelRegion::EdgeState>
octreeToGridParticle(const octomap::OcTree* octree,
                     const Eigen::Vector3d& minExtent,
                     const Eigen::Vector3d& maxExtent,
                     cv::RNG& rng);

class Object;

/// From object octrees to a particle representing the whole state
[[deprecated]]
mps::VoxelRegion::VertexLabels objectsToVoxelLabel(const std::map<ObjectIndex, std::unique_ptr<Object>>& objects,
                                                         const Eigen::Vector3d& roiMinExtent,
                                                         const Eigen::Vector3d& roiMaxExtent);

}

#endif //SRC_VOXELREGION_H
