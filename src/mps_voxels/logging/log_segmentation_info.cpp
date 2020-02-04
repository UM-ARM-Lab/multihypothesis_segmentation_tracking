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

#include "mps_voxels/logging/log_segmentation_info.h"
#include "mps_voxels/logging/log_cv_mat.h"
#include "mps_voxels/util/macros.h"

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

#define LOG_IMAGE_MESSAGE(var_name) \
	activeChannels.insert(channel + "/" STRINGIFY(var_name) ); \
	log(channel + "/" STRINGIFY(var_name) , toMessage(msg.var_name, header));
template <>
void DataLog::log<SegmentationInfo>(const std::string& channel, const SegmentationInfo& msg)
{
	std_msgs::Header header; header.stamp = msg.t; header.frame_id = msg.frame_id;
	APPLY_TO_ALL(LOG_IMAGE_MESSAGE, rgb, depth, ucm2, labels2, centroids2, stats2, display_contours, labels)

	activeChannels.insert(channel + "/objectness_segmentation"); \
	log(channel + "/objectness_segmentation", *msg.objectness_segmentation->toImageMsg());
}

#define LOAD_IMAGE_MESSAGE(var_name) \
	load(channel + "/" STRINGIFY(var_name) , im); msg.var_name = fromMessage(im);
template <>
bool DataLog::load<SegmentationInfo>(const std::string& channel, SegmentationInfo& msg)
{
	sensor_msgs::Image im;
	APPLY_TO_ALL(LOAD_IMAGE_MESSAGE, rgb, depth, ucm2, labels2, centroids2, stats2, display_contours, labels)
	msg.t = im.header.stamp; msg.frame_id = im.header.frame_id;

	load(channel + "/objectness_segmentation" , im);
	msg.objectness_segmentation = cv_bridge::toCvCopy(im);
	return true;
}

}
