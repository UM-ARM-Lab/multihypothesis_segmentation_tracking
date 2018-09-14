//
// Created by arprice on 9/13/18.
//

#include "mps_voxels/VictorManipulator.h"

#include <victor_hardware_interface/SetControlMode.h>

namespace hal = victor_hardware_interface;

std::vector<double> getQHome(robot_model::JointModelGroup* arm)
{
	if (arm->getName().find("left") != std::string::npos)
	{
		return std::vector<double>{0, 0, 0, -M_PI/4.0, -M_PI/2.0, -M_PI/2.0, -0.82};
	}
	else if (arm->getName().find("right") != std::string::npos)
	{
		return std::vector<double>{0, 0, 0, -M_PI/4.0, -M_PI/2.0, M_PI/2.0, 2.41};
	}
	else
	{
		throw std::runtime_error("Unknown arm.");
	}
}

VictorManipulator::VictorManipulator(ros::NodeHandle& nh,
                                     robot_model::RobotModelPtr _pModel,
                                     robot_model::JointModelGroup* _arm,
                                     robot_model::JointModelGroup* _gripper,
                                     std::string _palmName)
	: Manipulator(_pModel, _arm, _gripper, _palmName), qHome(getQHome(_arm))
{
	// TODO: nh.param
	std::string serviceName = "";
	if (arm->getName().find("left") != std::string::npos)
	{
		std::cerr << "Configuring left arm." << std::endl;
		serviceName = "/left_arm/set_control_mode_service";
	}
	else if (arm->getName().find("right") != std::string::npos)
	{
		std::cerr << "Configuring right arm." << std::endl;
		serviceName = "/right_arm/set_control_mode_service";
	}
	else
	{
		throw std::runtime_error("Unknown arm.");
	}

	configurationClient = nh.serviceClient<hal::SetControlMode>(serviceName);
	if (!configurationClient.waitForExistence(ros::Duration(3)))
	{
		ROS_WARN("Robot configuration server not connected.");
	}

}

bool VictorManipulator::configureHardware()
{
	hal::SetControlModeRequest req;
	req.new_control_mode.control_mode.mode = hal::ControlMode::JOINT_IMPEDANCE;
	req.new_control_mode.joint_path_execution_params.joint_relative_velocity = 0.2;
	req.new_control_mode.joint_path_execution_params.joint_relative_acceleration = 0.5;

	req.new_control_mode.joint_impedance_params.joint_damping.joint_1 = 0.7;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_2 = 0.7;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_3 = 0.7;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_4 = 0.7;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_5 = 0.7;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_6 = 0.7;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_7 = 0.7;

	req.new_control_mode.joint_impedance_params.joint_stiffness.joint_1 = 100.0;
	req.new_control_mode.joint_impedance_params.joint_stiffness.joint_2 = 100.0;
	req.new_control_mode.joint_impedance_params.joint_stiffness.joint_3 = 50.0;
	req.new_control_mode.joint_impedance_params.joint_stiffness.joint_4 = 50.0;
	req.new_control_mode.joint_impedance_params.joint_stiffness.joint_5 = 50.0;
	req.new_control_mode.joint_impedance_params.joint_stiffness.joint_6 = 50.0;
	req.new_control_mode.joint_impedance_params.joint_stiffness.joint_7 = 20.0;

	req.new_control_mode.header.frame_id = "victor_root";
	req.new_control_mode.header.stamp = ros::Time::now();

	hal::SetControlModeResponse resp;
	bool callSucceeded = configurationClient.call(req, resp);
	return callSucceeded && resp.success;
}