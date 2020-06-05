//
// Created by kunhuang on 7/23/19.
//

#include "mps_simulation/GazeboMocap.h"

#include <gazebo_msgs/GetWorldProperties.h>
#include <gazebo_msgs/GetModelState.h>

#include <tf_conversions/tf_eigen.h>

namespace mps
{

Eigen::Isometry3d randomTransform(std::mt19937& gen, const double tScale)
{
	std::uniform_real_distribution<> uni(-1.0, std::nextafter(1.0, std::numeric_limits<double>::max()));
	Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
#if EIGEN_VERSION_AT_LEAST(3, 3, 0)
	auto rand_fn = [&](){return uni(gen);};
#else
	auto rand_fn = [&](float){return uni(gen);};
#endif
	T.rotate(Eigen::Quaterniond(Eigen::Vector4d::NullaryExpr(rand_fn)).normalized());
	T.translation() = tScale * Eigen::Vector3d::NullaryExpr(rand_fn);
	return T;
}

GazeboMocap::GazeboMocap(size_t randomSeed)
{
	nh = std::make_unique<ros::NodeHandle>();
	tb = std::make_unique<tf::TransformBroadcaster>();
	worldStateClient = nh->serviceClient<gazebo_msgs::GetWorldProperties>("/gazebo/get_world_properties");
	modelStateClient = nh->serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	worldStateClient.waitForExistence(ros::Duration(5));
	modelStateClient.waitForExistence(ros::Duration(5));

	std::mt19937 gen(randomSeed);
	tf::transformEigenToTF(randomTransform(gen), gazeboTmocap);
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