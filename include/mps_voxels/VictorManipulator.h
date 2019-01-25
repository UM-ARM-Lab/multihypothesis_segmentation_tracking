//
// Created by arprice on 9/13/18.
//

#ifndef MPS_VICTORMANIPULATOR_H
#define MPS_VICTORMANIPULATOR_H

#include "mps_voxels/Manipulator.h"

#include <ros/node_handle.h>
#include <ros/service_client.h>

class VictorManipulator : public Manipulator
{
public:
	VictorManipulator(ros::NodeHandle& nh,
	                  robot_model::RobotModelPtr _pModel,
	                  robot_model::JointModelGroup* _arm,
	                  robot_model::JointModelGroup* _gripper,
	                  std::string _palmName);

	bool configureHardware() override;
	bool isGrasping(const robot_state::RobotState& currentState) const override;

	ros::ServiceClient configurationClient;

	const std::vector<double> qHome;
};

#endif // MPS_VICTORMANIPULATOR_H
