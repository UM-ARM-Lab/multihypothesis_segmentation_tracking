//
// Created by kunhuang on 2/4/20.
//

#ifndef ARMLAB_WS_LOG_CV_ROI_H
#define ARMLAB_WS_LOG_CV_ROI_H

#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/logging/DataLog.h"

namespace mps
{

template <>
void DataLog::log<cv::Rect>(const std::string& channel, const cv::Rect& msg);

template <>
bool DataLog::load<cv::Rect>(const std::string& channel, cv::Rect& msg);

}

#endif //ARMLAB_WS_LOG_CV_ROI_H
