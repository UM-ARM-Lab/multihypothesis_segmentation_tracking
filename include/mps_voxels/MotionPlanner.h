//
// Created by arprice on 9/4/18.
//

#ifndef PROJECT_MOTIONPLANNER_H
#define PROJECT_MOTIONPLANNER_H

#include "mps_voxels/Manipulator.h"
#include <octomap/OcTree.h>
#include <tf/transform_broadcaster.h>

class MotionPlanner
{
public:
	std::vector<std::shared_ptr<Manipulator>> manipulators;

	Eigen::Isometry3d robotTworld;
	std::string globalFrame;
	Eigen::Vector3d minExtent, maxExtent;

	bool visualize = true;
	std::shared_ptr<tf::TransformBroadcaster> broadcaster;

	std::default_random_engine rng;

	trajectory_msgs::JointTrajectory planPush(const octomap::OcTree*, const robot_state::RobotState& currentState, const Eigen::Affine3d& pushFrame);
	trajectory_msgs::JointTrajectory planDrag(const octomap::OcTree*, const robot_state::RobotState& currentState, const Eigen::Affine3d& pushFrame);
};

#endif // PROJECT_MOTIONPLANNER_H
