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

#ifndef MPS_MOTIONPLANNER_H
#define MPS_MOTIONPLANNER_H

#include "mps_voxels/Scene.h"
#include "mps_voxels/planning/Motion.h"

#include <moveit/planning_scene/planning_scene.h>

namespace mps
{

class ObjectSampler
{
public:
	using Pose = typename Scene::Pose;

	bool succeeded = false;
	ObjectIndex id;
	octomap::point3d samplePoint; ///< Sampled shadow point in world coordinates
	octomap::point3d cameraOrigin; ///< Camera origin in world coordinates
	octomath::Vector3 ray;
	octomap::point3d collision; ///< Center of collision voxel in world coordinates

	ObjectSampler() = default;
	explicit ObjectSampler(const Scenario* scenario, const OccupancyData* scene);
	explicit operator bool() const { return succeeded; }
};

collision_detection::WorldPtr computeCollisionWorld(const OccupancyData& occupancy);

class MotionPlanner
{
public:
	using Pose = typename Scene::Pose;
	using PoseSequence = std::vector<Pose, Eigen::aligned_allocator<Pose>>;
	using RankedPose = std::pair<double, MotionPlanner::Pose>;

	struct Introspection
	{
		ObjectSampler objectSampleInfo;
	};

	MotionPlanner(std::shared_ptr<const Scenario> _scenario, std::shared_ptr<const OccupancyData> _occupancy);

	std::shared_ptr<const Scenario> scenario;
	std::shared_ptr<const OccupancyData> env;

	static const std::string CLUTTER_NAME;
	collision_detection::WorldConstPtr collisionWorld;
	planning_scene::PlanningSceneConstPtr planningScene;
	planning_scene::PlanningSceneConstPtr computePlanningScene(bool useCollisionObjects = true);

	double reward(const robot_state::RobotState& robotState, const Motion* motion) const;

	collision_detection::AllowedCollisionMatrix gripperEnvironmentACM(const std::shared_ptr<Manipulator>& manipulator) const;
	bool gripperEnvironmentCollision(const std::shared_ptr<Manipulator>& manipulator, const robot_state::RobotState& robotState) const;
	bool addPhysicalObstructions(const std::shared_ptr<Manipulator>& manipulator, const robot_state::RobotState& robotState, OccupancyData::ObstructionList& collisionObjects) const;
	bool addVisualObstructions(const ObjectIndex target, OccupancyData::ObstructionList& collisionObjects) const;

	std::shared_ptr<Motion> samplePush(const robot_state::RobotState& robotState, Introspection* = nullptr) const;
	std::shared_ptr<Motion> sampleSlide(const robot_state::RobotState& robotState, Introspection* = nullptr) const;
	std::shared_ptr<Motion> pick(const robot_state::RobotState& robotState, const ObjectIndex target, OccupancyData::ObstructionList& collisionObjects) const;

	std::shared_ptr<Motion> recoverCrash(const robot_state::RobotState& robotState, const robot_state::RobotState& recoveryState) const;
};

struct ComparePoses
{
	using RankedPose = MotionPlanner::RankedPose;
	bool operator()(const RankedPose& a, const RankedPose& b ) { return a.first < b.first; };
};

}

#endif // MPS_MOTIONPLANNER_H
