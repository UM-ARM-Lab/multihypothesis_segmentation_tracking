//
// Created by kunhuang on 2/21/20.
//

#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_voxel_region.h"
#include "mps_voxels/logging/log_segmentation_info.h"
#include <std_msgs/Int32MultiArray.h>

namespace mps
{

template <>
void DataLog::log<OccupancyData>(const std::string& channel, const OccupancyData& msg)
{
	activeChannels.insert(channel + "/voxelRegion");
	log(channel + "/voxelRegion", *msg.voxelRegion);

	std_msgs::Int32MultiArray vs;
	vs.data = msg.vertexState;
	activeChannels.insert(channel + "/vertexState");
	log(channel + "/vertexState", vs);
}

template <>
OccupancyData DataLog::load<OccupancyData>(const std::string& channel)
{
	auto voxelRegion = std::make_shared<VoxelRegion>(load<VoxelRegion>(channel + "/voxelRegion"));
	auto vs = load<std_msgs::Int32MultiArray>(channel + "/vertexState");

	return OccupancyData(voxelRegion, vs.data);
}

}
