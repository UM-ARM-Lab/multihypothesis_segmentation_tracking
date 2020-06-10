//
// Created by kunhuang on 6/10/20.
//

#ifndef SRC_LOG_SIAMMASK_H
#define SRC_LOG_SIAMMASK_H

#include "mps_voxels/logging/DataLog.h"
#include <opencv2/core.hpp>

namespace mps
{

using SiamMaskData = std::map<uint16_t, std::map<ros::Time, cv::Mat>>;

template <>
void DataLog::log<SiamMaskData>(const std::string& channel, const SiamMaskData& msg);

template <>
SiamMaskData DataLog::load<SiamMaskData>(const std::string& channel);

}

#endif //SRC_LOG_SIAMMASK_H
