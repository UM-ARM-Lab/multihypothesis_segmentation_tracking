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

#ifndef MPS_VOXEL_RECOMBINATION_H
#define MPS_VOXEL_RECOMBINATION_H

#include "mps_voxels/VoxelRegion.h"

#include <boost/bimap.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/filtered_graph.hpp>
#include <boost/thread/shared_mutex.hpp>

#include <random>

namespace mps
{
std::pair<VoxelRegion::VertexLabels, std::map<std::pair<int, int>, double>>
computeSegmentationGraph(const VoxelRegion& vox, const std::vector<const VoxelRegion::VertexLabels*>& particles);

using ObjectInstance = std::pair<ParticleIndex, ObjectIndex>;

struct ConflictEdgeProperties
{
	int numOverlaps = 0;
};

using ConflictGraph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, ObjectInstance, ConflictEdgeProperties>;
using NodeLookup = boost::bimap<ObjectInstance, ConflictGraph::vertex_descriptor>;
using component_mapping_t = std::map<ConflictGraph::vertex_descriptor, unsigned long>;
using VertexComponentMap = std::shared_ptr<component_mapping_t>;
using ComponentOrdering = std::vector<std::map<ConflictGraph::vertex_descriptor, ConflictGraph::vertex_descriptor>>;

using ComponentGraph = boost::filtered_graph<ConflictGraph, std::function<bool(ConflictGraph::edge_descriptor)>, std::function<bool(ConflictGraph::vertex_descriptor)>>;

class VoxelConflictResolver
{
public:
	std::shared_ptr<const VoxelRegion> vox;

	ConflictGraph G;
	NodeLookup vertexLookup;
	VertexComponentMap components;
	std::vector<std::vector<ConflictGraph::vertex_descriptor>> componentToMembers;
	std::vector<ComponentGraph> componentGraphs;
	std::map<ObjectInstance, int> objectSizes;

	boost::shared_mutex graphMtx;
	boost::shared_mutex lookupMtx;

	VoxelConflictResolver(std::shared_ptr<const VoxelRegion> vox, const std::vector<const VoxelRegion::VertexLabels*>& particles);

	ConflictGraph::vertex_descriptor getOrAddVertex(const ObjectInstance& inst, ConflictGraph& g, NodeLookup& lookup);

	inline std::string node_name(const ObjectInstance& obj) const { return "n" + std::to_string(obj.first.id) + "_" + std::to_string(obj.second.id); }

	void print(std::ostream& out) const;

	ComponentOrdering
	sampleStructure(std::mt19937& re) const;

	VoxelRegion::VertexLabels
	sampleGeometry(const std::vector<const VoxelRegion::VertexLabels*>& particles, const ComponentOrdering& structures, std::mt19937& re) const;
};


}

#endif //MPS_VOXEL_RECOMBINATION_H
