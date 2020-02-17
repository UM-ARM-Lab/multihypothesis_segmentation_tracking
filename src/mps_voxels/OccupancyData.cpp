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

namespace mps
{

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
