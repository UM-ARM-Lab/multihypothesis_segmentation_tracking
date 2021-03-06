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

#include "mps_voxels/logging/log_cv_mat.h"

#include <opencv2/imgcodecs.hpp>

namespace mps
{

std::string cvType2Str(int type)
{
	std::string r;
	switch (type)
	{
	case CV_8UC1:  return sensor_msgs::image_encodings::MONO8;
	case CV_8UC3:  return sensor_msgs::image_encodings::BGR8;
	case CV_16UC1: return sensor_msgs::image_encodings::MONO16;
	default: ; // Do nothing
	}

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans+'0');

	return r;
}


sensor_msgs::Image
ros_message_conversion<cv::Mat>::toMessage(const cv::Mat& t, const std::nullptr_t)
{
	return *cv_bridge::CvImage(std_msgs::Header(), cvType2Str(t.type()), t).toImageMsg();
}

sensor_msgs::Image
ros_message_conversion<cv::Mat>::toMessage(const cv::Mat& t, const std_msgs::Header& h)
{
	return *cv_bridge::CvImage(h, cvType2Str(t.type()), t).toImageMsg();
}

cv::Mat
ros_message_conversion<cv::Mat>::fromMessage(const sensor_msgs::Image& t)
{
	return cv_bridge::toCvCopy(t, t.encoding)->image;
}

sensor_msgs::CameraInfo
ros_message_conversion<image_geometry::PinholeCameraModel>::toMessage(const image_geometry::PinholeCameraModel& t, const std::nullptr_t)
{
	return t.cameraInfo();
}

image_geometry::PinholeCameraModel
ros_message_conversion<image_geometry::PinholeCameraModel>::fromMessage(const sensor_msgs::CameraInfo& t)
{
	image_geometry::PinholeCameraModel msg;
	msg.fromCameraInfo(t);
	return msg;
}


sensor_msgs::CompressedImage toMaskMessage(const cv::Mat& mask, const std_msgs::Header& header)
{
	std::vector<int> compression_params;
	compression_params.push_back(cv::IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(9);
	compression_params.push_back(cv::IMWRITE_PNG_BILEVEL);
	compression_params.push_back(1);

	sensor_msgs::CompressedImage imgMsg;
	imgMsg.header = header;
	imgMsg.format = "png";
	cv::imencode(".png", mask, imgMsg.data, compression_params);

	return imgMsg;
}

cv::Mat fromCompressedMessage(const sensor_msgs::CompressedImage& msg)
{
	// NB: This seems to correctly handle 1-bit pngcv::IMREAD_GRAYSCALE
	return cv::imdecode(msg.data, cv::IMREAD_UNCHANGED); //
}

template <>
void DataLog::log<image_geometry::PinholeCameraModel>(const std::string& channel, const image_geometry::PinholeCameraModel& msg)
{
	log(channel, msg.cameraInfo());
}

template <>
image_geometry::PinholeCameraModel DataLog::load<image_geometry::PinholeCameraModel>(const std::string& channel)
{
	return fromMessage(load<sensor_msgs::CameraInfo>(channel));
}

template <>
cv::Mat DataLog::load<cv::Mat>(const std::string& channel)
{
	try
	{
		return fromCompressedMessage(load<sensor_msgs::CompressedImage>(channel));
	}
	catch (const rosbag::BagException&)
	{
		return fromMessage(load<sensor_msgs::Image>(channel));
	}
}

}
