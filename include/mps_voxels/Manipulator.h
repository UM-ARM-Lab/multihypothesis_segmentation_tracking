//
// Created by arprice on 9/3/18.
//

#ifndef PROJECT_MANIPULATOR_H
#define PROJECT_MANIPULATOR_H

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

class Manipulator
{
public:
	robot_model::RobotModelPtr pModel;
	robot_model::JointModelGroup* arm;
	robot_model::JointModelGroup* gripper;
	std::string palmName;

	Manipulator(robot_model::RobotModelPtr _pModel,
	            robot_model::JointModelGroup* _arm,
	            robot_model::JointModelGroup* _gripper,
	            std::string _palmName);

	double stateCost(const std::vector<double>& q1) const;

	double transitionCost(const std::vector<double>& q1, const double t1, const std::vector<double>& q2, const double t2) const;


	bool interpolate(const robot_state::RobotState& currentState, const robot_state::RobotState& toState,
	                 trajectory_msgs::JointTrajectory& cmd, const int INTERPOLATE_STEPS = 15) const;

	std::vector<std::vector<double>> IK(const Eigen::Affine3d& worldGoalPose, const Eigen::Affine3d& robotTworld, const robot_state::RobotState& currentState) const;

	bool cartesianPath(const std::vector<Eigen::Affine3d>& worldGoalPoses, const Eigen::Affine3d& robotTworld,
	                   const robot_state::RobotState& currentState, trajectory_msgs::JointTrajectory& cmd) const;

protected:
	mutable std::vector<double> qHome;
};

#endif // PROJECT_MANIPULATOR_H
