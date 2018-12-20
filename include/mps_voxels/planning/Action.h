//
// Created by arprice on 9/7/18.
//

#ifndef MPS_PLANNING_ACTION_H
#define MPS_PLANNING_ACTION_H

#include "mps_voxels/planning/State.h"
#include <trajectory_msgs/JointTrajectory.h>
#include <moveit_msgs/Grasp.h>
#include <victor_hardware_interface/Robotiq3FingerCommand.h>

class Action
{
public:
	using Pose = State::Pose;
	using Poses = std::vector<Pose, Eigen::aligned_allocator<Pose>>;
	virtual ~Action() = default;
};

class JointTrajectoryAction : public Action
{
public:
	trajectory_msgs::JointTrajectory cmd;
	Poses palm_trajectory;
//	std::string jointGroupName;
};

class GripperCommandAction : public Action
{
public:
	using GraspDefinition = victor_hardware_interface::Robotiq3FingerCommand; // moveit_msgs::Grasp
	GraspDefinition grasp;
//	Poses palm_trajectory;
	std::string jointGroupName;
};

class CompositeAction : public Action
{
public:
	std::vector<std::shared_ptr<Action>> actions;
	int primaryAction = -1; ///< Useful for chains where most actions are setting up for a main one
};

#endif // MPS_PLANNING_ACTION_H
