/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef SRC_LOG_CV_MAT_H
#define SRC_LOG_CV_MAT_H

#include "mps_voxels/logging/conversion.hpp"
#include "mps_voxels/logging/DataLog.h"

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <string>


namespace mps
{

std::string cvType2Str(int type);

MESSAGE_CONVERSION_BOILERPLATE_AUX(cv::Mat, sensor_msgs::Image, std_msgs::Header)

MESSAGE_CONVERSION_BOILERPLATE(image_geometry::PinholeCameraModel, sensor_msgs::CameraInfo)

template <>
void DataLog::log<image_geometry::PinholeCameraModel>(const std::string& channel, const image_geometry::PinholeCameraModel& msg);

template <>
image_geometry::PinholeCameraModel DataLog::load<image_geometry::PinholeCameraModel>(const std::string& channel);

template <>
cv::Mat DataLog::load<cv::Mat>(const std::string& channel);

}

#endif // SRC_LOG_CV_MAT_H
