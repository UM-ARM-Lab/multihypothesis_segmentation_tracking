//
// Created by arprice on 9/4/18.
//

#ifndef MPS_MOTIONPLANNER_H
#define MPS_MOTIONPLANNER_H

#include "mps_voxels/Scene.h"
#include "mps_voxels/planning/Motion.h"

#include <moveit/planning_scene/planning_scene.h>

class ObjectSampler
{
public:
	using Pose = typename Scene::Pose;

	Scene* env;

	bool sampleObject(ObjectIndex& id, Pose& pushFrame) const;
};

class MotionPlanner
{
public:
	using Pose = typename Scene::Pose;
	using PoseSequence = std::vector<Pose, Eigen::aligned_allocator<Pose>>;
	using RankedPose = std::pair<double, MotionPlanner::Pose>;

	Scene* env;

	ObjectSampler objectSampler;

	planning_scene::PlanningSceneConstPtr planningScene;
	planning_scene::PlanningSceneConstPtr computePlanningScene(bool useCollisionObjects = true);

	double reward(const robot_state::RobotState& robotState, const Motion* motion) const;

	collision_detection::AllowedCollisionMatrix gripperEnvironmentACM(const std::shared_ptr<Manipulator>& manipulator) const;
	bool gripperEnvironmentCollision(const std::shared_ptr<Manipulator>& manipulator, const robot_state::RobotState& robotState) const;
	bool addPhysicalObstructions(const std::shared_ptr<Manipulator>& manipulator, const robot_state::RobotState& robotState, Scene::ObstructionList& collisionObjects) const;
	bool addVisualObstructions(const ObjectIndex target, Scene::ObstructionList& collisionObjects) const;

	std::shared_ptr<Motion> samplePush(const robot_state::RobotState& robotState) const;
	std::shared_ptr<Motion> sampleSlide(const robot_state::RobotState& robotState) const;
	std::shared_ptr<Motion> pick(const robot_state::RobotState& robotState, const ObjectIndex target, Scene::ObstructionList& collisionObjects) const;

	std::shared_ptr<Motion> recoverCrash(const robot_state::RobotState& robotState, const robot_state::RobotState& recoveryState) const;
};

struct ComparePoses
{
	using RankedPose = MotionPlanner::RankedPose;
	bool operator()(const RankedPose& a, const RankedPose& b ) { return a.first < b.first; };
};

#endif // MPS_MOTIONPLANNER_H
