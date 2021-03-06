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

#include "mps_voxels/Manipulator.h"
#include "mps_voxels/util/assert.h"
#include "mps_voxels/Viterbi.hpp"

#include <moveit/planning_scene/planning_scene.h>

#include <memory>

Manipulator::Manipulator(robot_model::RobotModelPtr _pModel,
                         robot_model::JointModelGroup* _arm,
                         robot_model::JointModelGroup* _gripper,
                         std::string _palmName)
	: pModel(std::move(_pModel)), arm(_arm), gripper(_gripper), palmName(std::move(_palmName))
{
	MPS_ASSERT(arm->getVariableCount() == arm->getActiveJointModels().size());
	qMin.resize(arm->getVariableCount());
	qMax.resize(arm->getVariableCount());
	qMid.resize(arm->getVariableCount());
	const std::vector<const moveit::core::JointModel::Bounds*>& bounds = arm->getActiveJointModelsBounds();

	for (size_t j = 0; j < bounds.size(); ++j)
	{
		MPS_ASSERT(bounds[j]->size() == 1);
		const auto& b = bounds[j]->front();

		qMin[j] = b.min_position_;
		qMax[j] = b.max_position_;
		qMid[j] = (qMin[j] + qMax[j])/2.0;
	}
}

double Manipulator::stateCost(const std::vector<double>& q1) const
{
	const std::vector<double> JOINT_WEIGHTS(q1.size(), 1.0);
	double cost = 0;
	for (size_t j = 0; j < q1.size(); ++j)
	{
		double midpointDistanceNormalized = (q1[j]-qMid[j])/(qMax[j]-qMin[j]);
		cost += midpointDistanceNormalized*midpointDistanceNormalized;

//		double homeDistance = fabs(q1[j] - qHome[j]);
//		cost += homeDistance*homeDistance;
	}
	return cost/2.0;
}

double Manipulator::transitionCost(const std::vector<double>& q1, const double t1, const std::vector<double>& q2, const double t2) const
{
	const std::vector<double> JOINT_WEIGHTS(q1.size(), 5.0);
	MPS_ASSERT(JOINT_WEIGHTS.size() == q1.size());
	MPS_ASSERT(JOINT_WEIGHTS.size() == q2.size());
	MPS_ASSERT(t2 > t1);
	double cost = 0;
	for (size_t j = 0; j < q1.size(); ++j)
	{
//		cost += pow(fabs(q1[j] - q2[j]), 2) * JOINT_WEIGHTS[j];
		cost += fabs(q1[j] - q2[j]) * JOINT_WEIGHTS[j];
	}
	return cost/(t2-t1);
}


bool Manipulator::interpolate(const Pose& from, const Pose& to, PoseSequence& sequence, const int INTERPOLATE_STEPS ) const
{
	sequence.reserve(INTERPOLATE_STEPS);
	Eigen::Quaterniond qStart(from.linear()), qEnd(to.linear());
	for (int i = 0; i<INTERPOLATE_STEPS; ++i)
	{
		double t = i/static_cast<double>(INTERPOLATE_STEPS-1);
		moveit::Pose interpPose = moveit::Pose::Identity();
		interpPose.translation() = ((1.0-t)*from.translation())+(t*to.translation());
		interpPose.linear() = qStart.slerp(t, qEnd).matrix();
		sequence.push_back(interpPose);
	}

	return true;
}


bool Manipulator::interpolate(const robot_state::RobotState& currentState, const robot_state::RobotState& toState,
                              trajectory_msgs::JointTrajectory& cmd, const int INTERPOLATE_STEPS,
                              planning_scene::PlanningSceneConstPtr scene) const
{
	planning_scene::PlanningSceneConstPtr ps;

	if (scene)
	{
		ps = scene;
	}
	else
	{
		ps = std::make_unique<planning_scene::PlanningScene>(pModel);
	}

	collision_detection::CollisionRequest collision_request;
	collision_detection::CollisionResult collision_result;

	ps->checkCollision(collision_request, collision_result, toState);

	if (collision_result.collision)
	{
		return false;
	}

	cmd.joint_names = arm->getActiveJointModelNames();
	cmd.points.resize(cmd.points.size() + INTERPOLATE_STEPS);

	// Verify the "plan" is collision-free
	for (int i = 0; i<INTERPOLATE_STEPS; ++i)
	{
		double t = i/static_cast<double>(INTERPOLATE_STEPS-1);
		robot_state::RobotState interpState(currentState);
		currentState.interpolate(toState, t, interpState, arm);

		ps->checkCollision(collision_request, collision_result, interpState);
		if (collision_result.collision)
		{
			return false;
		}

		interpState.copyJointGroupPositions(arm, cmd.points[i].positions);
		cmd.points[i].time_from_start = ros::Duration(t*3.0);
	}

	return true;
}

std::vector<std::vector<double>> Manipulator::IK(const moveit::Pose& worldGoalPose, const moveit::Pose& robotTworld, const robot_state::RobotState& currentState) const
{
	std::vector<std::vector<double>> solutions;
	const kinematics::KinematicsBaseConstPtr& solver = arm->getSolverInstance();
	MPS_ASSERT(solver.get());

	// NB: The (possibly dirty) frames in RobotState are not marked mutable, hence the const casting.
	moveit::Pose solverbaseTrobot = moveit::Pose::Identity();
	const_cast<robot_state::RobotState&>(currentState).updateLinkTransforms();
	const_cast<robot_state::RobotState&>(currentState).setToIKSolverFrame(solverbaseTrobot, solver);

	moveit::Pose solvertipTgoal = currentState.getFrameTransform(solver->getTipFrame()).inverse(Eigen::Isometry) * currentState.getFrameTransform(this->palmName);

	// Convert to solver frame
	moveit::Pose pt_solver = solverbaseTrobot * robotTworld * worldGoalPose * solvertipTgoal.inverse(Eigen::Isometry);

	std::vector<geometry_msgs::Pose> targetPoses;
	Eigen::Quaterniond q(pt_solver.linear());
	geometry_msgs::Pose pose;
	pose.position.x = pt_solver.translation().x();
	pose.position.y = pt_solver.translation().y();
	pose.position.z = pt_solver.translation().z();
	pose.orientation.x = q.x();
	pose.orientation.y = q.y();
	pose.orientation.z = q.z();
	pose.orientation.w = q.w();
	targetPoses.push_back(pose);

	std::vector<double> seed(arm->getVariableCount(), 0.0);
	currentState.copyJointGroupPositions(arm, seed);
	kinematics::KinematicsResult result; // NOLINT(cppcoreguidelines-pro-type-member-init)
	kinematics::KinematicsQueryOptions options;
	options.discretization_method = kinematics::DiscretizationMethod::ALL_DISCRETIZED;
	solver->getPositionIK(targetPoses, seed, solutions, result, options);

	return solutions;
}

bool Manipulator::cartesianPath(const PoseSequence& worldGoalPoses, const moveit::Pose& robotTworld,
                                const robot_state::RobotState& currentState, trajectory_msgs::JointTrajectory& cmd) const
{
	const auto stateCostFn = [&](const std::vector<double>& q){ return this->stateCost(q); };
	const auto transitionCostFn = [&](const std::vector<double>& q1, const double t1, const std::vector<double>& q2, const double t2){ return this->transitionCost(q1, t1, q2, t2); };

	std::vector<std::vector<std::vector<double>>> trellis;// 3d grid
	std::vector<double> times(worldGoalPoses.size());
//	double discretization = arm->getSolverInstance()->getSearchDiscretization();
//	if (0 == discretization) { discretization = 1; }
//	trellis.reserve(worldGoalPoses.size()*(int)(M_PI/discretization)*arm->getVariableCount());

	currentState.copyJointGroupPositions(arm, qHome);

	trellis.resize(worldGoalPoses.size());
	for (int i = 0; i < static_cast<int>(worldGoalPoses.size()); ++i)
	{
		times[i] = i;
		trellis[i] = IK(worldGoalPoses[i], robotTworld, currentState);
		if (trellis[i].empty())
		{
			ROS_WARN_STREAM("Unable to compute Cartesian trajectory: no IK solution found for (world frame):\n"
			<< worldGoalPoses[i].translation().transpose() << "\t" << Eigen::Quaterniond(worldGoalPoses[i].linear()).coeffs().transpose());
			return false;
		}
	}

	// Compute a reasonable cost threshold
	double dist = M_PI/16.0;
	std::vector<double> fromJoints(arm->getVariableCount(), 0.0), toJoints(arm->getVariableCount(), dist);
	double maxCost = transitionCost(fromJoints, times[0], toJoints, times[1]);

	std::vector<int> cheapestPath = mps::viterbi(trellis, times, stateCostFn, transitionCostFn, maxCost);
	if (cheapestPath.size() != worldGoalPoses.size())
	{
		ROS_ERROR_STREAM("Unable to compute Cartesian trajectory: no path found through solution graph.");
		return false;
	}

	cmd.points.clear();
	cmd.joint_names.clear();

	robot_state::RobotState startState(pModel);
	startState.setToDefaultValues();
	startState.setJointGroupPositions(arm, trellis[0][cheapestPath[0]]);
	startState.updateCollisionBodyTransforms();
//
//	int interpSteps = 15;
//	if (!interpolate(currentState, startState, cmd, interpSteps))
//	{
//		return false;
//	}

	cmd.points.reserve(worldGoalPoses.size());
	for (int i = 0; i < static_cast<int>(worldGoalPoses.size()); ++i)
	{
		int slnIdx = cheapestPath[i];
		double t = times[i];

		trajectory_msgs::JointTrajectoryPoint jtpt;
		jtpt.positions = trellis[i][slnIdx];
		jtpt.time_from_start = ros::Duration(t);
		cmd.points.push_back(jtpt);
	}
	cmd.joint_names = arm->getActiveJointModelNames();

	return true;
}

bool Manipulator::grasp(const robot_state::RobotState& /*currentState*/, moveit_msgs::Grasp& grasp) const
{
	grasp.grasp_posture.joint_names = gripper->getActiveJointModelNames();
	grasp.grasp_posture.points.resize(1);

	std::map<std::string, size_t> jointNameToIdx;
	for (const std::string& name : grasp.grasp_posture.joint_names)
	{
		jointNameToIdx.insert({name, jointNameToIdx.size()});
	}

//	grasp.grasp_posture.points.front().positions[]
	return false;
}