//
// Created by kunhuang on 2/21/20.
//

#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_voxel_region.h"
#include <std_msgs/Int32MultiArray.h>

namespace mps
{

mps_msgs::VoxelSegmentation
ros_message_conversion<OccupancyData>::toMessage(const OccupancyData& occupancy, const std::nullptr_t)
{
	mps_msgs::VoxelSegmentation msg;
	msg.region = ::mps::toMessage(*occupancy.voxelRegion);
	msg.data = occupancy.vertexState;
	return msg;
}

OccupancyData
ros_message_conversion<OccupancyData>::fromMessage(const mps_msgs::VoxelSegmentation& msg)
{
	return OccupancyData(std::make_shared<VoxelRegion>(::mps::fromMessage(msg.region)), msg.data);
}

template <>
void DataLog::log<OccupancyData>(const std::string& channel, const OccupancyData& msg)
{
	activeChannels.insert(channel);

	log(channel, toMessage(msg));
}

OccupancyData loadLegacyOccupancy(DataLog& log, const std::string& channel)
{
	auto voxelRegion = std::make_shared<VoxelRegion>(log.load<VoxelRegion>(channel + "/voxelRegion"));
	auto vs = log.load<std_msgs::Int32MultiArray>(channel + "/vertexState");

	return OccupancyData(voxelRegion, vs.data);
}

template <>
OccupancyData DataLog::load<OccupancyData>(const std::string& channel)
{
	rosbag::View view(*bag, TopicTypeQuery(channel, ros::message_traits::DataType<mps_msgs::VoxelSegmentation>::value()));
	if (view.size() == 0)
	{
		return loadLegacyOccupancy(*this, channel);
	}

	return fromMessage(load<mps_msgs::VoxelSegmentation>(channel));
}

}
