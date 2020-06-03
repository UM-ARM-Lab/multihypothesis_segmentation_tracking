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

#ifndef MPS_MANIPULATOR_H
#define MPS_MANIPULATOR_H

#include <mps_voxels/moveit_pose_type.h>

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit/collision_detection/world.h>
#include <moveit/planning_scene/planning_scene.h>

#include <moveit_msgs/Grasp.h>
#include <trajectory_msgs/JointTrajectory.h>

#include <Eigen/StdVector>

class Manipulator
{
public:
	using Pose = moveit::Pose;
	using PoseSequence = std::vector<Pose, Eigen::aligned_allocator<Pose>>;

	robot_model::RobotModelPtr pModel;
	robot_model::JointModelGroup* arm;
	robot_model::JointModelGroup* gripper;
	std::string palmName;

	Manipulator(robot_model::RobotModelPtr _pModel,
	            robot_model::JointModelGroup* _arm,
	            robot_model::JointModelGroup* _gripper,
	            std::string _palmName);

	virtual
	bool configureHardware() { return true; }

	double stateCost(const std::vector<double>& q1) const;

	double transitionCost(const std::vector<double>& q1, const double t1, const std::vector<double>& q2, const double t2) const;

	bool interpolate(const Pose& from, const Pose& to, PoseSequence& sequence,
	                 const int INTERPOLATE_STEPS = 15) const;

	bool interpolate(const robot_state::RobotState& currentState, const robot_state::RobotState& toState,
	                 trajectory_msgs::JointTrajectory& cmd, const int INTERPOLATE_STEPS = 15,
	                 planning_scene::PlanningSceneConstPtr world = planning_scene::PlanningSceneConstPtr()) const;

	std::vector<std::vector<double>> IK(const moveit::Pose& worldGoalPose, const moveit::Pose& robotTworld, const robot_state::RobotState& currentState) const;

	bool cartesianPath(const PoseSequence& worldGoalPoses, const moveit::Pose& robotTworld,
	                   const robot_state::RobotState& currentState, trajectory_msgs::JointTrajectory& cmd) const;

	bool grasp(const robot_state::RobotState& currentState, moveit_msgs::Grasp& grasp) const;

	virtual
	bool isGrasping(const robot_state::RobotState&) const { return true; }

	virtual
	std::vector<double> getGripperOpenJoints() const { return {}; }

protected:
	mutable std::vector<double> qHome;
	std::vector<double> qMin;
	std::vector<double> qMax;
	std::vector<double> qMid;
};

#endif // MPS_MANIPULATOR_H
