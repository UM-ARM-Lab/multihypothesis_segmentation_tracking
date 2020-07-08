//
// Created by kunhuang on 2/21/20.
//

#include "mps_voxels/logging/log_voxel_region.h"
#include "mps_msgs/VoxelRegion.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
#include <ros/console.h>

namespace mps
{

mps_msgs::VoxelRegion
ros_message_conversion<VoxelRegion>::toMessage(const VoxelRegion& region, const std::nullptr_t)
{
	mps_msgs::VoxelRegion msg;
	msg.resolution = region.resolution;
	msg.min.x = region.regionMin.x();
	msg.min.y = region.regionMin.y();
	msg.min.z = region.regionMin.z();
	msg.max.x = region.regionMax.x();
	msg.max.y = region.regionMax.y();
	msg.max.z = region.regionMax.z();
	msg.frame_id = region.frame_id;
	return msg;
}

VoxelRegion
ros_message_conversion<VoxelRegion>::fromMessage(const mps_msgs::VoxelRegion& msg)
{
	return VoxelRegion(msg.resolution,
	                   {msg.min.x, msg.min.y, msg.min.z},
	                   {msg.max.x, msg.max.y, msg.max.z},
	                   msg.frame_id);
}

template <>
void DataLog::log<VoxelRegion>(const std::string& channel, const VoxelRegion& region)
{
	activeChannels.insert(channel);

	log(channel, toMessage(region));
}

VoxelRegion loadLegacyRegion(DataLog& log, const std::string& channel)
{
	auto res = log.load<std_msgs::Float64>(channel + "/resolution");
	auto rMin = log.load<geometry_msgs::Vector3>(channel + "/regionMin" );
	auto rMax = log.load<geometry_msgs::Vector3>(channel + "/regionMax" );

	std::string frame_id = "";
	try
	{
		frame_id = log.load<std::string>(channel + "/frame_id");
	}
	catch (const std::exception& ex)
	{
		ROS_ERROR_STREAM("Failed to load a frame_id: '" << ex.what() << "'");
	}

	return VoxelRegion(res.data, {rMin.x, rMin.y, rMin.z}, {rMax.x, rMax.y, rMax.z}, frame_id);
}

template <>
VoxelRegion DataLog::load<VoxelRegion>(const std::string& channel)
{
	rosbag::View view(*bag, TopicTypeQuery(channel, ros::message_traits::DataType<mps_msgs::VoxelRegion>::value()));
	if (view.size() == 0)
	{
		return loadLegacyRegion(*this, channel);
	}

	return fromMessage(load<mps_msgs::VoxelRegion>(channel));
}

}
