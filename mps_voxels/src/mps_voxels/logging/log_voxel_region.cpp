//
// Created by kunhuang on 2/21/20.
//

#include "mps_voxels/logging/log_voxel_region.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <ros/console.h>

namespace mps
{

template <>
void DataLog::log<VoxelRegion>(const std::string& channel, const VoxelRegion& msg)
{
	std_msgs::Float64 res;
	res.data = msg.resolution;
	activeChannels.insert(channel + "/resolution");
	log(channel + "/resolution", res);

	geometry_msgs::Vector3 rMin;
	rMin.x = msg.regionMin.x();
	rMin.y = msg.regionMin.y();
	rMin.z = msg.regionMin.z();
	activeChannels.insert(channel + "/regionMin");
	log(channel + "/regionMin", rMin);

	geometry_msgs::Vector3 rMax;
	rMax.x = msg.regionMax.x();
	rMax.y = msg.regionMax.y();
	rMax.z = msg.regionMax.z();
	activeChannels.insert(channel + "/regionMax");
	log(channel + "/regionMax", rMax);

	log<const std::string&>(channel + "/frame_id", msg.frame_id);
}

template <>
VoxelRegion DataLog::load<VoxelRegion>(const std::string& channel)
{
	auto res = load<std_msgs::Float64>(channel + "/resolution");
	auto rMin = load<geometry_msgs::Vector3>(channel + "/regionMin" );
	auto rMax = load<geometry_msgs::Vector3>(channel + "/regionMax" );

	std::string frame_id = "";
	try
	{
		frame_id = load<std::string>(channel + "/frame_id");
	}
	catch (const std::exception& ex)
	{
		ROS_ERROR_STREAM("Failed to load a frame_id: '" << ex.what() << "'");
	}

	return VoxelRegion(res.data, {rMin.x, rMin.y, rMin.z}, {rMax.x, rMax.y, rMax.z}, frame_id);
}

}
