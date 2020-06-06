//
// Created by kunhuang on 2/4/20.
//

#ifndef ARMLAB_WS_LOG_CV_ROI_H
#define ARMLAB_WS_LOG_CV_ROI_H

#include "mps_voxels/logging/conversion.hpp"
#include "mps_voxels/logging/DataLog.h"

#include <opencv2/core.hpp>
#include <sensor_msgs/RegionOfInterest.h>

namespace mps
{

MESSAGE_CONVERSION_BOILERPLATE(cv::Rect, sensor_msgs::RegionOfInterest)

template <>
void DataLog::log<cv::Rect>(const std::string& channel, const cv::Rect& msg);

template <>
cv::Rect DataLog::load<cv::Rect>(const std::string& channel);

}

#endif //ARMLAB_WS_LOG_CV_ROI_H
