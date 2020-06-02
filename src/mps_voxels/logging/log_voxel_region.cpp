//Process finished with exit code 139 (interrupted by signal 11: SIGSEGV)

// Created by kunhuang on 2/21/20.
//

#include "mps_voxels/logging/log_voxel_region.h"
#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3.h>
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
}

template <>
bool DataLog::load<VoxelRegion>(const std::string& channel, VoxelRegion& msg)
{
	std_msgs::Float64 res;
	load(channel + "/resolution" , res);

	geometry_msgs::Vector3 rMin;
	load(channel + "/regionMin" , rMin);

	geometry_msgs::Vector3 rMax;
	load(channel + "/regionMax" , rMax);

	msg = VoxelRegion(res.data, {rMin.x, rMin.y, rMin.z}, {rMax.x, rMax.y, rMax.z});

	return true;
}

}
