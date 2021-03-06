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

#include "mps_voxels/logging/log_sensor_history.h"
#include "mps_voxels/logging/log_cv_mat.h"

namespace mps
{

template <>
void DataLog::log<SensorHistoryBuffer>(const std::string& channel, const SensorHistoryBuffer& msg)
{
	std_msgs::Header header = msg.cameraModel.cameraInfo().header;

	activeChannels.insert(channel + "/camera_model");
	activeChannels.insert(channel + "/rgb");
	activeChannels.insert(channel + "/depth");
	activeChannels.insert(channel + "/joint");
	activeChannels.insert(channel + "/tf");
	activeChannels.insert(channel + "/tf_static");

	log(channel + "/camera_model", msg.cameraModel);
	for (const auto& pair : msg.rgb)
	{
		header.stamp = pair.first;
		log(channel + "/rgb", toMessage(pair.second->image, header));
	}
	for (const auto& pair : msg.depth)
	{
		header.stamp = pair.first;
		log(channel + "/depth", toMessage(pair.second->image, header));
	}
	for (const auto& pair : msg.joint)
	{
		log(channel + "/joint", *pair.second);
	}
	for (const auto& t : msg.tf_raw)
	{
		log(channel + "/tf", *t);
	}
	for (const auto& t : msg.tf_static_raw)
	{
		log(channel + "/tf_static", *t);
	}

}

template <>
SensorHistoryBuffer DataLog::load<SensorHistoryBuffer>(const std::string& channel)
{
	SensorHistoryBuffer msg;
	msg.cameraModel = load<image_geometry::PinholeCameraModel>(channel + "/camera_model");

	std::vector<sensor_msgs::Image> rgbs;
	loadAll(channel + "/rgb", rgbs);
	for (const auto& m : rgbs)
	{
		msg.rgb.insert(std::make_pair(m.header.stamp, cv_bridge::toCvCopy(m)));
		assert(msg.rgb.rbegin()->second->image.type() == CV_8UC3);
	}

	std::vector<sensor_msgs::Image> depths;
	loadAll(channel + "/depth", depths);
	for (const auto& m : depths)
	{
		msg.depth.insert(std::make_pair(m.header.stamp, cv_bridge::toCvCopy(m)));
		assert(msg.depth.rbegin()->second->image.type() == CV_16UC1);
	}

	std::vector<sensor_msgs::JointState> joints;
	loadAll(channel + "/joint", joints);
	for (const auto& m : joints)
	{
		auto js = sensor_msgs::JointStatePtr(new sensor_msgs::JointState);
		*js = m;
		msg.joint.insert(std::make_pair(m.header.stamp, js));
	}

	// TODO: make the history length reasonable, based on the images
	msg.tfs = std::make_shared<tf2_ros::Buffer>(ros::Duration(100*tf2_ros::Buffer::DEFAULT_CACHE_TIME));
	loadAll(channel + "/tf", msg.tf_raw);
	for (const auto& m : msg.tf_raw)
	{
		for (const auto & transform : m->transforms)
		{
			msg.tfs->setTransform(transform, "", false);
		}
	}
	loadAll(channel + "/tf_static", msg.tf_static_raw);
	for (const auto& m : msg.tf_static_raw)
	{
		for (const auto & transform : m->transforms)
		{
			msg.tfs->setTransform(transform, "", true);
		}
	}

	return msg;
}

}
