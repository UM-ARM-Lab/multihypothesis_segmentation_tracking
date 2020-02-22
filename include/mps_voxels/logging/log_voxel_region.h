//
// Created by kunhuang on 2/21/20.
//

#ifndef SRC_LOG_VOXEL_REGION_H
#define SRC_LOG_VOXEL_REGION_H

#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/VoxelRegion.h"

namespace mps
{

template <>
void DataLog::log<VoxelRegion>(const std::string& channel, const VoxelRegion& msg);

template <>
bool DataLog::load<VoxelRegion>(const std::string& channel, VoxelRegion& msg);

}

#endif //SRC_LOG_VOXEL_REGION_H
