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

#include "mps_voxels/VictorManipulator.h"

#include <victor_hardware_interface/SetControlMode.h>

#include <utility>

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
	: Manipulator(std::move(_pModel), _arm, _gripper, std::move(_palmName)), qHome(getQHome(_arm))
{
	// TODO: nh.param
	std::string serviceName = "";
	if (arm->getName().find("left") != std::string::npos)
	{
		ROS_INFO_STREAM("Configuring left arm.");
		serviceName = "/left_arm/set_control_mode_service";
	}
	else if (arm->getName().find("right") != std::string::npos)
	{
		ROS_INFO_STREAM("Configuring right arm.");
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

	const double damping = 0.7;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_1 = damping;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_2 = damping;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_3 = damping;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_4 = damping;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_5 = damping;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_6 = damping;
	req.new_control_mode.joint_impedance_params.joint_damping.joint_7 = damping;

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

bool VictorManipulator::isGrasping(const robot_state::RobotState& currentState) const
{
//	for (const auto& n : currentState.getVariableNames()) { std::cerr << n << std::endl; }
//	std::cerr << "-----------------" << std::endl;
//	for (const auto& n : gripper->getActiveJointModelNames()) { std::cerr << n << std::endl; }

	std::vector<std::string> fingerJointNames;
	if (arm->getName().find("left") != std::string::npos)
	{
		fingerJointNames = { "victor_left_gripper_fingerA_joint_2", "victor_left_gripper_fingerB_joint_2", "victor_left_gripper_fingerC_joint_2" };
	}
	else if (arm->getName().find("right") != std::string::npos)
	{
		fingerJointNames = { "victor_right_gripper_fingerA_joint_2", "victor_right_gripper_fingerB_joint_2", "victor_right_gripper_fingerC_joint_2" };
	}
	else
	{
		throw std::runtime_error("Unknown arm.");
	}

	// NB: These are the values in radians, not the control signal from 0-1
	const double MIN_POS = 0.1; // 0.075;
	const double MAX_POS = 0.75; // 0.35;
	for (const auto& n : fingerJointNames)
	{
		double q = currentState.getVariablePosition(n);
		if (MIN_POS < q && q < MAX_POS)
		{
			return true;
		}
	}

	return false;
}

std::vector<double> VictorManipulator::getGripperOpenJoints() const
{
	return std::vector<double>(this->gripper->getVariableCount(), 0.0);
}