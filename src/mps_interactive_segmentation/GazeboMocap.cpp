//
// Created by kunhuang on 7/23/19.
//

#include "mps_interactive_segmentation/GazeboMocap.h"

#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>

#include <tf_conversions/tf_eigen.h>

namespace mps
{

Eigen::Isometry3d randomTransform(const double tScale)
{
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
	T.rotate(Eigen::Quaterniond(Eigen::Vector4d::Random()).normalized());
	T.translation() = tScale * Eigen::Vector3d::Random();
	return T;
}

GazeboMocap::GazeboMocap()
{
	nh = std::make_unique<ros::NodeHandle>();
	tb = std::make_unique<tf::TransformBroadcaster>();
	worldStateClient = nh->serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
	modelStateClient = nh->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	worldStateClient.waitForExistence(ros::Duration(5));
	modelStateClient.waitForExistence(ros::Duration(5));

	tf::transformEigenToTF(randomTransform(), gazeboTmocap);
	gazeboTmocap.frame_id_ = "world";
	gazeboTmocap.child_frame_id_ = mocap_frame_id;
}


std::vector<std::string> GazeboMocap::getModelNames()
{
	gazebo_msgs::GetWorldPropertiesRequest req;
	gazebo_msgs::GetWorldPropertiesResponse resp;
	bool result = worldStateClient.call(req, resp);

	if (!result) { throw std::runtime_error("Call to '" + worldStateClient.getService() + "' failed."); }
	if (!resp.success)  { throw std::runtime_error("Call to '" + worldStateClient.getService() + "' reported failure: '" + resp.status_message + "'"); }

	return resp.model_names;
}

bool GazeboMocap::getTransforms()
{
	for (const auto& pair : markerOffsets)
	{
		gazebo_msgs::GetModelStateRequest req;
		req.model_name = pair.first.gazeboName();
		//req.reference_frame = "";
		gazebo_msgs::GetModelStateResponse resp;
		bool result = modelStateClient.call(req, resp);

		if (!result)
		{
			ROS_ERROR_STREAM("Call to '" << modelStateClient.getService() << "' failed.");
			return false;
		}
		if (!resp.success)
		{
			ROS_ERROR_STREAM("Call to '" << modelStateClient.getService() << "' unable to look up "
			                             << "->" + req.model_name);
			return false;
		}

		tf::Transform gazeboTlink; tf::poseMsgToTF(resp.pose, gazeboTlink);
		tf::StampedTransform stf(gazeboTlink, resp.header.stamp, gazeboTmocap.frame_id_, pair.first.gazeboName());

		linkPoses[pair.first] = stf;
	}

	return true;
}

void GazeboMocap::sendTransforms(bool sendBaseFrame)
{
	// For debugging
	if (sendBaseFrame)
	{
		gazeboTmocap.stamp_ = ros::Time::now();
		tb->sendTransform(gazeboTmocap);
	}

	for (const auto& pair : markerOffsets)
	{
		const auto& linkTmarker = pair.second;

		tf::Transform mocapTmarker = gazeboTmocap.inverse() * linkPoses[pair.first] * linkTmarker;

		tb->sendTransform(tf::StampedTransform(mocapTmarker, ros::Time::now(), mocap_frame_id, "mocap_" + pair.first.tfName()));
	}
}

}