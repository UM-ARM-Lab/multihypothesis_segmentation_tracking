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

	octomap::OcTree* sceneOctree;
	std::map<octomap::point3d, int, vector_less_than<3, octomap::point3d>> coordToObject;
	std::vector<std::shared_ptr<octomap::OcTree>> completedSegments;
	octomap::point3d_collection occludedPts;

	static const std::string CLUTTER_NAME;
	mutable collision_detection::WorldPtr collisionWorld;
	collision_detection::WorldConstPtr getCollisionWorldConst() const;
	collision_detection::WorldPtr getCollisionWorld();


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

	PlanningEnvironment* env;

	ObjectSampler objectSampler;

	double reward(const Motion* motion) const;

	trajectory_msgs::JointTrajectory planPush(const octomap::OcTree*, const robot_state::RobotState& currentState, const Eigen::Affine3d& pushFrame);
	std::shared_ptr<Motion> sampleSlide(const robot_state::RobotState& robotState);
};

#endif // MPS_MOTIONPLANNER_H
