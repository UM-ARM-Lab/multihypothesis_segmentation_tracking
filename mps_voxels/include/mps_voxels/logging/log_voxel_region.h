//
// Created by kunhuang on 2/21/20.
//

#ifndef SRC_LOG_VOXEL_REGION_H
#define SRC_LOG_VOXEL_REGION_H

#include "mps_voxels/logging/conversion.hpp"
#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/VoxelRegion.h"

#include <mps_msgs/VoxelRegion.h>

namespace mps
{

MESSAGE_CONVERSION_BOILERPLATE(VoxelRegion, mps_msgs::VoxelRegion)

template <>
void DataLog::log<VoxelRegion>(const std::string& channel, const VoxelRegion& msg);

template <>
VoxelRegion DataLog::load<VoxelRegion>(const std::string& channel);

}

#endif //SRC_LOG_VOXEL_REGION_H
