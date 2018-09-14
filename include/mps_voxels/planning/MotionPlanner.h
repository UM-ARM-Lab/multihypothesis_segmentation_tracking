//
// Created by arprice on 9/4/18.
//

#ifndef MPS_MOTIONPLANNER_H
#define MPS_MOTIONPLANNER_H

#include "mps_voxels/planning/Motion.h"
#include "mps_voxels/Manipulator.h"
#include "mps_voxels/vector_less_than.h"

#include <moveit/collision_detection/world.h>
//#include <moveit/collision_detection/collision_world.h>

#include <octomap/OcTree.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/StdVector>
#include <moveit/planning_scene/planning_scene.h>

class PlanningEnvironment
{
public:
	using Pose = Eigen::Affine3d;

	bool visualize = true;
	std::shared_ptr<tf::TransformBroadcaster> broadcaster;

	std::default_random_engine rng;

	Pose worldTrobot;
	Pose worldTcamera;
	std::string worldFrame;
	std::string cameraFrame;
	Eigen::Vector3d minExtent, maxExtent;

	std::vector<std::shared_ptr<Manipulator>> manipulators;
	std::map<std::string, std::shared_ptr<Manipulator>> jointToManipulator;

	std::vector<std::pair<std::shared_ptr<shapes::Shape>, Pose>> staticObstacles;

	octomap::OcTree* sceneOctree;
	std::map<int, octomap::point3d_collection> objectToShadow;
	std::map<octomap::point3d, int, vector_less_than<3, octomap::point3d>> coordToObject;
	std::map<octomap::point3d, int, vector_less_than<3, octomap::point3d>> surfaceCoordToObject;
	std::vector<std::shared_ptr<octomap::OcTree>> completedSegments;
	std::vector<std::shared_ptr<shapes::Mesh>> approximateSegments;
	octomap::point3d_collection occludedPts;

	std::set<int> obstructions;

	static const std::string CLUTTER_NAME;
	collision_detection::WorldConstPtr collisionWorld;
	collision_detection::WorldPtr computeCollisionWorld();


	enum { NeedsToAlign = (sizeof(Pose)%16)==0 };
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

class ObjectSampler
{
public:
	using Pose = typename PlanningEnvironment::Pose;

	PlanningEnvironment* env;

	bool sampleObject(int& id, Pose& pushFrame) const;
};

class MotionPlanner
{
public:
	using Pose = typename PlanningEnvironment::Pose;
	using PoseSequence = std::vector<Pose, Eigen::aligned_allocator<Pose>>;
	using RankedPose = std::pair<double, MotionPlanner::Pose>;

	PlanningEnvironment* env;

	ObjectSampler objectSampler;

	planning_scene::PlanningSceneConstPtr planningScene;
	planning_scene::PlanningSceneConstPtr computePlanningScene(bool useCollisionObjects = true);

	double reward(const robot_state::RobotState& robotState, const Motion* motion) const;

	collision_detection::AllowedCollisionMatrix gripperEnvironmentACM(const std::shared_ptr<Manipulator>& manipulator) const;
	bool gripperEnvironmentCollision(const std::shared_ptr<Manipulator>& manipulator, const robot_state::RobotState& robotState) const;

	std::shared_ptr<Motion> samplePush(const robot_state::RobotState& robotState) const;
	std::shared_ptr<Motion> sampleSlide(const robot_state::RobotState& robotState) const;
	std::shared_ptr<Motion> pick(const robot_state::RobotState& robotState, const int target, std::set<int>& collisionObjects) const;

	std::shared_ptr<Motion> recoverCrash(const robot_state::RobotState& robotState, const robot_state::RobotState& recoveryState) const;
};

struct ComparePoses
{
	using RankedPose = MotionPlanner::RankedPose;
	bool operator()(const RankedPose& a, const RankedPose& b ) { return a.first < b.first; };
};

#endif // MPS_MOTIONPLANNER_H
