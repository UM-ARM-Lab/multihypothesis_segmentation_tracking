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
#include "mps_voxels/DisjointSetForest.hpp"
#include "mps_voxels/Object.h"

#include <ros/console.h>

#include <opencv2/core.hpp>

#include <map>

namespace mps
{

VoxelRegion::VoxelRegion(boost::array<std::size_t, Dimensions> dims, double res, Eigen::Vector3d rmin, Eigen::Vector3d rmax) // NOLINT(hicpp-member-init)
	: resolution(res), regionMin(std::move(rmin)), regionMax(std::move(rmax)), m_dimension_lengths(dims)
{
	precalculate();
	regionMinSnapped = {snapCoord(resolution, rmin.x()), snapCoord(resolution, rmin.y()), snapCoord(resolution, rmin.z())};
}

size_t VoxelRegion::getEdgeIndex(VoxelRegion::vertex_descriptor a, VoxelRegion::vertex_descriptor b)
{
	// Normalize the ordering
	auto i = index_of(a);
	auto j = index_of(b);
	if (j < i) { std::swap(a, b); }

	// Check for adjacency (boost::edge(a, b) does this, but return the descriptor and not the ID.)
	int dist = 0;
	for (size_t d = 0; d < dimensions(); ++d)
	{
		dist += std::abs(static_cast<long>(a[d]) - static_cast<long>(b[d]));
	}
	if (dist != 1) { throw std::logic_error("Grid vertices for edge lookup are not adjacent."); }

	// Return the edge
	return index_of({a, b});
}

VoxelRegion::VertexLabels VoxelRegion::components(VoxelRegion::EdgeState& edges) const
{
	DisjointSetForest<int> dsf(num_vertices());

	for (Grid::edges_size_type e = 0; e < num_edges(); ++e)
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
	for (Grid::edges_size_type e = 0; e < num_edges(); ++e)
	{
		const Grid::edge_descriptor edge = edge_at(e);
		auto i = index_of(source(edge, *this));
		auto j = index_of(target(edge, *this));

		edges[e] = (dsf.nodes[i] == dsf.nodes[j]);
	}

	// Flatten and relabel tree as root labels
	std::map<ObjectIndex, unsigned> sizes;
	for (size_t i = 0; i < dsf.nodes.size(); ++i)
	{
		int label = dsf.getAncestor(i);
		dsf.nodes[i] = label;
		sizes[ObjectIndex(label)]++;
	}

	for (size_t i = 0; i < dsf.nodes.size(); ++i)
	{
		ObjectIndex label(dsf.getAncestor(i));
		if (sizes[label] == 1)
		{
			dsf.nodes[i] = -1;
		}
	}

	return dsf.nodes;
}

void VoxelRegion::precalculate()
{
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

VoxelRegion::vertex_descriptor
VoxelRegion::next(VoxelRegion::vertex_descriptor vertex, std::size_t dimension_index,
                        VoxelRegion::vertices_size_type distance) const
{

	vertices_size_type new_position =
		vertex[dimension_index] + distance;

	// Stop at the end of this dimension if necessary.
	new_position =
		(std::min)(new_position,
		           vertices_size_type(length(dimension_index) - 1));

	vertex[dimension_index] = new_position;

	return (vertex);
}

VoxelRegion::vertex_descriptor
VoxelRegion::previous(VoxelRegion::vertex_descriptor vertex, std::size_t dimension_index,
                            VoxelRegion::vertices_size_type distance) const
{

	// We're assuming that vertices_size_type is unsigned, so we
	// need to be careful about the math.
	vertex[dimension_index] =
		(distance > vertex[dimension_index]) ? 0 : vertex[dimension_index] - distance;

	return (vertex);
}

VoxelRegion::vertices_size_type VoxelRegion::index_of(VoxelRegion::vertex_descriptor vertex) const
{

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

VoxelRegion::vertex_descriptor
VoxelRegion::vertex_at(VoxelRegion::vertices_size_type vertex_index) const
{

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

VoxelRegion::edge_descriptor VoxelRegion::edge_at(VoxelRegion::edges_size_type edge_index) const
{

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

VoxelRegion::edges_size_type VoxelRegion::index_of(VoxelRegion::edge_descriptor edge) const
{
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

std::map<ObjectIndex, std::shared_ptr<octomap::OcTree>>
VoxelRegion::vertexLabelToOctrees(const VertexLabels& vlabels, const std::set<ObjectIndex>& uniqueObjectLabels)
{
	std::map<ObjectIndex, std::shared_ptr<octomap::OcTree>> labelToOcTreeLookup;
	for (auto label : uniqueObjectLabels)
	{
		labelToOcTreeLookup.emplace(label, std::make_shared<octomap::OcTree>(resolution));
	}

	for (int i = 0; i < (int)vlabels.size(); ++i)
	{
		if (vlabels[i] >= 0)
		{
			vertex_descriptor query = vertex_at(i);
			Eigen::Vector3d pos = coordinate_of(query);

			labelToOcTreeLookup[ObjectIndex(vlabels[i])]->updateNode(pos.x(), pos.y(), pos.z(), true, true);
//			labelToOcTreeLookup[ObjectIndex(vlabels[i])]->setNodeValue(pos.x(), pos.y(), pos.z(), 1.0);
		}
	}

	for (auto& pair : labelToOcTreeLookup)
	{
		pair.second->updateInnerOccupancy();
	}

	return labelToOcTreeLookup;
}

mps::VoxelRegion::vertex_descriptor
roiToGrid(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& roiMax)
{
	auto minCoord = snap(roiMin, octree);
	auto maxCoord = snap(roiMax, octree);

	mps::VoxelRegion::vertex_descriptor dims;
	for (int i = 0; i < 3; ++i)
	{
		dims[i] = ceil((maxCoord[i]-minCoord[i])/octree->getResolution()) + 1;
	}

	return dims;
}

mps::VoxelRegion::vertex_descriptor
roiToVoxelRegion(const double& resolution, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& roiMax)
{
	Eigen::Vector3d minCoord = {snapCoord(resolution, roiMin.x()), snapCoord(resolution, roiMin.y()), snapCoord(resolution, roiMin.z())};
	Eigen::Vector3d maxCoord = {snapCoord(resolution, roiMax.x()), snapCoord(resolution, roiMax.y()), snapCoord(resolution, roiMax.z())};

	mps::VoxelRegion::vertex_descriptor dims;
	for (int i = 0; i < 3; ++i)
	{
		dims[i] = ceil((maxCoord[i]-minCoord[i])/resolution) + 1;
	}

	return dims;
}

mps::VoxelRegion::vertex_descriptor
coordToGrid(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& query)
{
	auto minCoord = snap(roiMin, octree);
	auto queryCoord = snap(query, octree);

	mps::VoxelRegion::vertex_descriptor dims;
	for (int i = 0; i < 3; ++i)
	{
		dims[i] = round((queryCoord[i]-minCoord[i])/octree->getResolution());
	}

	return dims;
}

mps::VoxelRegion::vertex_descriptor coordToVertexDesc(const double& resolution, const Eigen::Vector3d& roiMin, const Eigen::Vector3d& query)
{
	Eigen::Vector3d minCoord = {snapCoord(resolution, roiMin.x()), snapCoord(resolution, roiMin.y()), snapCoord(resolution, roiMin.z())};
	Eigen::Vector3d queryCoord = {snapCoord(resolution, query.x()), snapCoord(resolution, query.y()), snapCoord(resolution, query.z())};

	mps::VoxelRegion::vertex_descriptor dims;
	for (int i = 0; i < 3; ++i)
	{
		dims[i] = round((queryCoord[i]-minCoord[i])/resolution);
	}
	return dims;
}

Eigen::Vector3d gridToCoord(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin,
                            const mps::VoxelRegion::vertex_descriptor& query)
{
	auto minCoord = snap(roiMin, octree);
	return minCoord + octree->getResolution() * (Eigen::Map<const Eigen::Matrix<std::size_t, 3, 1>>(query.data()).cast<double>());
}

Eigen::Vector3d vertexDescpToCoord(const double& resolution, const Eigen::Vector3d& roiMin, const mps::VoxelRegion::vertex_descriptor& query)
{
	Eigen::Vector3d minCoord = {snapCoord(resolution, roiMin.x()), snapCoord(resolution, roiMin.y()), snapCoord(resolution, roiMin.z())};
	return minCoord + resolution * (Eigen::Map<const Eigen::Matrix<std::size_t, 3, 1>>(query.data()).cast<double>());
}

bool isOccupied(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin,
                const mps::VoxelRegion::vertex_descriptor& query)
{
	auto coord = gridToCoord(octree, roiMin, query);
	octomap::OcTreeNode* node = octree->search(coord.x(), coord.y(), coord.z());
	if (node)
	{
		return node->getOccupancy() > octree->getOccupancyThres();
	}
	return false;
}

std::pair<bool, double> sampleIsOccupied(const octomap::OcTree* octree, const Eigen::Vector3d& roiMin,
                                         const mps::VoxelRegion::vertex_descriptor& query, cv::RNG& rng)
{
	auto coord = gridToCoord(octree, roiMin, query);
	octomap::OcTreeNode* node = octree->search(coord.x(), coord.y(), coord.z());

	bool a;
	double pa;
	if (node)
	{
		a = node->getOccupancy() > rng.uniform(0.0, 1.0);
		pa = a ? node->getOccupancy() : (1.0-node->getOccupancy());
	}
	else
	{
		a = false;
		pa = 1.0;
	}
	return {a, pa};
}

mps::VoxelRegion::EdgeState
octreeToGrid(const octomap::OcTree* octree, const Eigen::Vector3d& minExtent, const Eigen::Vector3d& maxExtent)
{
	mps::VoxelRegion::vertex_descriptor dims = roiToGrid(octree, minExtent, maxExtent);
	mps::VoxelRegion vox(dims, octree->getResolution(), minExtent, maxExtent);

	mps::VoxelRegion::EdgeState edges(vox.num_edges());

	for (mps::VoxelRegion::edges_size_type e = 0; e < vox.num_edges(); ++e)
	{
		const mps::VoxelRegion::edge_descriptor edge = vox.edge_at(e);
		auto i = source(edge, vox);
		auto j = target(edge, vox);

		bool a = isOccupied(octree, minExtent, i);
		bool b = isOccupied(octree, minExtent, j);

		edges[e] = (a && b);
	}

	return edges;
}

std::pair<double, mps::VoxelRegion::EdgeState>
octreeToGridParticle(const octomap::OcTree* octree, const Eigen::Vector3d& minExtent, const Eigen::Vector3d& maxExtent,
                     cv::RNG& rng)
{
	mps::VoxelRegion::vertex_descriptor dims = roiToGrid(octree, minExtent, maxExtent);
	mps::VoxelRegion vox(dims, octree->getResolution(), minExtent, maxExtent);

	mps::VoxelRegion::EdgeState edges(vox.num_edges());

	double logOdds = 0;
	for (mps::VoxelRegion::edges_size_type e = 0; e < vox.num_edges(); ++e)
	{
		const mps::VoxelRegion::edge_descriptor edge = vox.edge_at(e);
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

mps::VoxelRegion::VertexLabels objectsToVoxelLabel(const std::map<ObjectIndex, std::unique_ptr<Object>>& objects,
                                                         const Eigen::Vector3d& roiMinExtent,
                                                         const Eigen::Vector3d& roiMaxExtent)
{
	mps::VoxelRegion::vertex_descriptor dims = roiToGrid(objects.begin()->second->occupancy.get(), roiMinExtent, roiMaxExtent);
	mps::VoxelRegion vox(dims, objects.begin()->second->occupancy->getResolution(), roiMinExtent, roiMaxExtent);

	mps::VoxelRegion::VertexLabels res(vox.num_vertices(), -1);

	int label = 0;
	for (auto& pair : objects)
	{
		auto obj = pair.second.get();
		assert(obj);
		mps::VoxelRegion::vertex_descriptor objDims = roiToGrid(obj->occupancy.get(), obj->minExtent.cast<double>(), obj->maxExtent.cast<double>());
		mps::VoxelRegion objVS(objDims, obj->occupancy->getResolution(), obj->minExtent.cast<double>(), obj->maxExtent.cast<double>());
		mps::VoxelRegion::vertex_descriptor objMin = coordToGrid(obj->occupancy.get(), roiMinExtent, obj->minExtent.cast<double>());

		for (mps::VoxelRegion::vertices_size_type v = 0; v < objVS.num_vertices(); ++v)
		{
			auto query = objVS.vertex_at(v);
			if ( isOccupied(obj->occupancy.get(), obj->minExtent.cast<double>(), query) )
			{
				mps::VoxelRegion::vertex_descriptor target;
				target[0] = query[0] + objMin[0];
				target[1] = query[1] + objMin[1];
				target[2] = query[2] + objMin[2];
				mps::VoxelRegion::vertices_size_type index = vox.index_of(target);
				if (index >= vox.num_vertices()) ROS_ERROR_STREAM("objects are outside voxel region!!!");
				else
					res[index] = label;
			}
		}
		label ++;
	}
	return res;
}

VoxelRegion::VertexLabels
VoxelRegion::objectsToSubRegionVoxelLabel(const std::map<ObjectIndex, std::unique_ptr<Object>>& objects,
                                          const Eigen::Vector3d& subRegionMinExtent)
{
	mps::VoxelRegion::VertexLabels res(num_vertices(), -1);
	mps::VoxelRegion::vertex_descriptor subRegionMinVD = coordToVertexDesc(resolution, regionMin, subRegionMinExtent);

	int label = 0;
	for (auto& pair : objects)
	{
		auto obj = pair.second.get();
		assert(obj);
		mps::VoxelRegion::vertex_descriptor objDims = roiToGrid(obj->occupancy.get(), obj->minExtent.cast<double>(), obj->maxExtent.cast<double>());
		mps::VoxelRegion objVS(objDims, obj->occupancy->getResolution(), obj->minExtent.cast<double>(), obj->maxExtent.cast<double>());
		mps::VoxelRegion::vertex_descriptor objMin = coordToGrid(obj->occupancy.get(), subRegionMinExtent, obj->minExtent.cast<double>());

		for (mps::VoxelRegion::vertices_size_type v = 0; v < objVS.num_vertices(); ++v)
		{
			auto query = objVS.vertex_at(v);
			if ( isOccupied(obj->occupancy.get(), obj->minExtent.cast<double>(), query) )
			{
				mps::VoxelRegion::vertex_descriptor target;
				target[0] = query[0] + objMin[0] + subRegionMinVD[0];
				target[1] = query[1] + objMin[1] + subRegionMinVD[1];
				target[2] = query[2] + objMin[2] + subRegionMinVD[2];
				mps::VoxelRegion::vertices_size_type index = index_of(target);
				if (index >= num_vertices()) ROS_ERROR_STREAM("objects are outside voxel region!!!");
				else
					res[index] = label;
			}
		}
		label ++;
	}
	return res;
}

}