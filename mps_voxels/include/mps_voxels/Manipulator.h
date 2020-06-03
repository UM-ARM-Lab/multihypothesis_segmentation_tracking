//
// Created by arprice on 9/3/18.
//

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
