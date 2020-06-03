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

	activeChannels.insert(channel + "/segInfo");
	log(channel + "/segInfo", *msg.segInfo);
}

template <>
bool DataLog::load<OccupancyData>(const std::string& channel, OccupancyData& msg)
{
	//// in case the voxelRegion is not initialized
	const double resolution = 1.0;
	Eigen::Vector3d ROImaxExtent(1.0, 1.0, 1.0);
	Eigen::Vector3d ROIminExtent(-1.0, -1.0, -1.0);
	msg.voxelRegion = std::make_shared<VoxelRegion>(resolution,
	                                                ROIminExtent,
	                                                ROImaxExtent);
	load(channel + "/voxelRegion", *msg.voxelRegion);

	std_msgs::Int32MultiArray vs;
	load(channel + "/vertexState", vs);
	msg.vertexState = vs.data;

	load(channel + "/segInfo", *msg.segInfo);
	return true;
}

}
