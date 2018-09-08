//
// Created by arprice on 9/7/18.
//

#ifndef MPS_PLANNING_ACTION_H
#define MPS_PLANNING_ACTION_H

#include "mps_voxels/planning/State.h"
#include <trajectory_msgs/JointTrajectory.h>

class Action
{
public:
	using Pose = State::Pose;
	using Poses = State::Poses;
	virtual ~Action() = default;
};

class JointTrajectoryAction : public Action
{
public:
	trajectory_msgs::JointTrajectory cmd;
	Poses palm_trajectory;
};

class GripperCommandAction : public Action
{

};

#endif // MPS_PLANNING_ACTION_H
