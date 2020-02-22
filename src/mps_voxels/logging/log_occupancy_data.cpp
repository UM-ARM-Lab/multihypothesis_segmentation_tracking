//
// Created by kunhuang on 2/21/20.
//

#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_voxel_region.h"
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
bool DataLog::load<OccupancyData>(const std::string& channel, OccupancyData& msg)
{
	//// in case the voxelRegion is not initialized
	const double resolution = 0.010;
	Eigen::Vector3f ROImaxExtent(0.4f, 0.6f, 0.5f);
	Eigen::Vector3f ROIminExtent(-0.4f, -0.6f, -0.020f);
	mps::VoxelRegion::vertex_descriptor dims = roiToVoxelRegion(resolution,
	                                                            ROIminExtent.cast<double>(),
	                                                            ROImaxExtent.cast<double>());
	msg.voxelRegion = std::make_shared<VoxelRegion>(dims, resolution,
	                                                ROIminExtent.cast<double>(),
	                                                ROImaxExtent.cast<double>());
	load(channel + "/voxelRegion", *msg.voxelRegion);

	std_msgs::Int32MultiArray vs;
	load(channel + "/vertexState", vs);
	msg.vertexState = vs.data;

	return true;
}

}
