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

#include "mps_voxels/logging/DataLog.h"

#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

#include <string>


namespace mps
{

std::string cvType2Str(int type);

template <typename T>
struct ros_message_conversion {};

template <typename T>
struct ros_message_type_of {};

/**
 * @tparam T Main type to be converted to message
 * @tparam Aux Auxiliary type containing any extra info (particularly headers)
 * @param t Object to convert
 * @param a Aux data object (defaults to null pointer)
 * @return
 */
template <typename T, typename Aux = std::nullptr_t>
auto toMessage(T t, Aux a = nullptr)
{
	return ros_message_conversion<std::remove_cv_t<std::remove_reference_t<T>>>::toMessage(t, a);
}

template <typename T>
auto fromMessage(T t)
{
	return ros_message_conversion<typename ros_message_type_of<std::remove_cv_t<std::remove_reference_t<T>>>::CppType>::fromMessage(t);
}


template<>
struct ros_message_type_of<sensor_msgs::Image> { using CppType = cv::Mat; };

template<>
struct ros_message_conversion<cv::Mat>
{
	using MsgType = sensor_msgs::Image;
	using CppType = cv::Mat;

	static
	MsgType toMessage(const CppType& t, const std::nullptr_t);

	static
	MsgType toMessage(const CppType& t, const std_msgs::Header& h);

	static
	CppType fromMessage(const MsgType& t);
};

template <>
void DataLog::log<image_geometry::PinholeCameraModel>(const std::string& channel, const image_geometry::PinholeCameraModel& msg);

template <>
bool DataLog::load<image_geometry::PinholeCameraModel>(const std::string& channel, image_geometry::PinholeCameraModel& msg);

}

#endif // SRC_LOG_CV_MAT_H
