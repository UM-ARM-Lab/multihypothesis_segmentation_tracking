//
// Created by kunhuang on 2/21/20.
//

#ifndef SRC_LOG_OCCUPANCY_DATA_H
#define SRC_LOG_OCCUPANCY_DATA_H

#include "mps_voxels/logging/conversion.hpp"
#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/OccupancyData.h"

#include <mps_msgs/VoxelSegmentation.h>

namespace mps
{

MESSAGE_CONVERSION_BOILERPLATE(OccupancyData, mps_msgs::VoxelSegmentation)

template <>
void DataLog::log<OccupancyData>(const std::string& channel, const OccupancyData& msg);

template <>
OccupancyData DataLog::load<OccupancyData>(const std::string& channel);

}

#endif //SRC_LOG_OCCUPANCY_DATA_H
