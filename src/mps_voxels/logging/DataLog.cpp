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

#include "mps_voxels/logging/DataLog.h"

#include <ros/console.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace mps
{

std::unique_ptr<DataLog> DataLog::instance;

DataLog::DataLog(const std::string& filename, const ChannelSet& channels, const rosbag::bagmode::BagMode mode)
{
	auto path = (filename.empty() ? getDefaultPath() : filename);
	if (!fs::exists(path) && mode == rosbag::BagMode::Read)
	{
		throw std::runtime_error("Attempting to read from nonexistant file '" + path + "'.");
	}
	bag = std::make_unique<rosbag::Bag>(path, mode);

	activeChannels = channels;

	ROS_INFO_STREAM("Logging to '" << bag->getFileName() << "'");
}

DataLog::~DataLog()
{
	bag->close();
}

std::string DataLog::getDefaultPath() const
{
	fs::path temp_dir = fs::temp_directory_path();
	fs::path temp_file = temp_dir / fs::path("log.bag");
	return temp_file.string();
}

ros::Time DataLog::getTime() const
{
	if (ros::Time::isValid())
	{
		return ros::Time::now();
	}
	else
	{
		std::chrono::system_clock::time_point tp = std::chrono::system_clock::now();
		std::time_t tt = std::chrono::system_clock::to_time_t(tp);
		return ros::Time(tt);
	}
}

template <>
void DataLog::log<const std::string&>(const std::string& channel, const std::string& msg)
{
	if (activeChannels.find(channel) == activeChannels.end()) { return; }
	std_msgs::String s;
	s.data = msg;
	log(channel, s);
}

template <>
void DataLog::log<const std::vector<char>>(const std::string& channel, const std::vector<char>& msg)
{
	if (activeChannels.find(channel) == activeChannels.end()) { return; }
	std_msgs::ByteMultiArray bytes;
	std::copy(msg.begin(), msg.end(), std::back_inserter(bytes.data));
	log(channel, bytes);
}

}
