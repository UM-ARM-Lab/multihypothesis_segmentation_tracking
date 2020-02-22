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

#include "mps_voxels/OccupancyData.h"
#include "mps_voxels/moveit_pose_type.h"
#include "mps_voxels/Scene.h"

#include "mps_voxels/LocalOctreeServer.h" // for setBBox()
#include "mps_voxels/shape_utils.h"

namespace mps
{

OccupancyData::OccupancyData(std::shared_ptr<VoxelRegion> region)
	: voxelRegion(std::move(region)),
	  vertexState(voxelRegion->num_vertices(), -1),
	  edgeState(voxelRegion->num_edges(), false)
{

}

OccupancyData::OccupancyData(const std::shared_ptr<Scene>& scene,
                             std::shared_ptr<VoxelRegion> region,
                             VoxelRegion::VertexLabels state)
	: voxelRegion(std::move(region)),
	  vertexState(std::move(state)),
	  edgeState(voxelRegion->num_edges(), false),
	  uniqueObjectLabels(getUniqueObjectLabels(vertexState))
{
	// There's currently ~1.5M edges. Reserve this for a sparser state representation
	// Build the edge representation
//	for (VoxelRegion::edges_size_type e = 0; e < voxelRegion->num_edges(); ++e)
//	{
//		const VoxelRegion::edge_descriptor edge = voxelRegion->edge_at(e);
//		auto i = voxelRegion->index_of(source(edge, voxelRegion));
//		auto j = voxelRegion->index_of(target(edge, voxelRegion));
//
//		edgeState[e] = (vertexState[i] == vertexState[j]);
//	}

	// Allocate the object representation
	for (const auto& id : uniqueObjectLabels)
	{
		std::shared_ptr<octomap::OcTree> subtree(scene->sceneOctree->create());
		subtree->setProbMiss(0.05);
		subtree->setProbHit(0.95);
		setBBox(scene->minExtent, scene->maxExtent, subtree.get());
		auto res = objects.emplace(id, std::make_unique<Object>(id, subtree));
		res.first->second->minExtent = Eigen::Vector3f::Constant(std::numeric_limits<float>::max());
		res.first->second->maxExtent = Eigen::Vector3f::Constant(std::numeric_limits<float>::lowest());
	}

	// Populate the object representation
	for (mps::VoxelRegion::vertices_size_type v = 0; v < voxelRegion->num_vertices(); ++v)
	{
		const auto label = vertexState[v];
		if (label < 0) { continue; }
		const auto descriptor = voxelRegion->vertex_at(v);
		const auto coordinate = voxelRegion->coordinate_of(descriptor).cast<float>();

		ObjectIndex objID(label);
		auto& obj = objects.at(objID);
		obj->occupancy->setNodeValue(coordinate.x(), coordinate.y(), coordinate.z(),
		                             std::numeric_limits<float>::max(), true);
		obj->minExtent = obj->minExtent.cwiseMin(coordinate);
		obj->maxExtent = obj->maxExtent.cwiseMax(coordinate);
	}

	// Prune and approximate shape data
	for (auto& pair : objects)
	{
		pair.second->occupancy->updateInnerOccupancy();
		pair.second->approximation = approximateShape(pair.second->occupancy.get());
	}
}

collision_detection::WorldPtr
computeCollisionWorld(const OccupancyData& occupancy)
{
	auto world = std::make_shared<collision_detection::World>();

	moveit::Pose robotTworld = occupancy.parentScene.lock()->worldTrobot.inverse(Eigen::Isometry);

	for (const auto& obstacle : occupancy.parentScene.lock()->scenario->staticObstacles)
	{
		world->addToObject(OccupancyData::CLUTTER_NAME, obstacle.first, robotTworld * obstacle.second);
	}

	// Use aliasing shared_ptr constructor
//	world->addToObject(CLUTTER_NAME,
//	                   std::make_shared<shapes::OcTree>(std::shared_ptr<octomap::OcTree>(std::shared_ptr<octomap::OcTree>{}, sceneOctree)),
//	                   robotTworld);

	for (const auto& obj : occupancy.objects)
	{
		const std::shared_ptr<octomap::OcTree>& segment = obj.second->occupancy;
		world->addToObject(std::to_string(obj.first.id), std::make_shared<shapes::OcTree>(segment), robotTworld);
	}

//	for (auto& approxSegment : approximateSegments)
//	{
//		world->addToObject(CLUTTER_NAME, approxSegment, robotTworld);
//	}

	return world;
}

}
