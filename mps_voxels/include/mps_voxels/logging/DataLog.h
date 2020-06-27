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

#ifndef SRC_DATALOGGER_H
#define SRC_DATALOGGER_H

#include <std_msgs/String.h>
#include <std_msgs/ByteMultiArray.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <chrono>
#include <mutex>
#include <set>
#include <unordered_set>


namespace mps
{

class TopicTypeQuery
{
public:
	TopicTypeQuery(const std::string& topic, const std::string& type) : topic_(topic), type_(type) {}

	bool operator()(rosbag::ConnectionInfo const* info) const
	{
		return (info->datatype == type_) && (info->topic == topic_);
	}

private:
	const std::string& topic_;
	const std::string& type_;
};

class DataLog
{
public:
	using ChannelSet = std::unordered_set<std::string>;
	ChannelSet activeChannels;
	std::unique_ptr<rosbag::Bag> bag;
	std::mutex mtx;

	explicit
	DataLog(const std::string& filename = "", const ChannelSet& channels = {}, const rosbag::BagMode mode = rosbag::bagmode::Write);
	~DataLog();

	template <class Msg>
	void log(const std::string& channel, const Msg& msg)
	{
		static_assert(ros::message_traits::IsMessage<Msg>::value, "Default log only defined for ROS message types.");
		std::lock_guard<std::mutex> lock(mtx);
//		if (activeChannels.find(channel) == activeChannels.end()) { return; }
		bag->write(channel, getTime(), msg);
	}

	template <class Msg>
	Msg load(const std::string& channel)
	{
		static_assert(ros::message_traits::IsMessage<Msg>::value, "Default load only defined for ROS message types.");
		std::lock_guard<std::mutex> lock(mtx);
		rosbag::View view(*bag, TopicTypeQuery(channel, ros::message_traits::DataType<Msg>::value()));

		// There may be many valid messages matching the request; we return the first one we find.
		for (const rosbag::MessageInstance& m : view)
		{
			auto p = m.instantiate<Msg>();
			if (p)
			{
				return *p;
			}
		}

		throw rosbag::BagException("Failed to load message of type '" +
		                           std::string(ros::message_traits::DataType<Msg>::value()) + "' from channel '" +
		                           channel + "' in file '" + this->bag->getFileName() + "'.");
//		return false;
	}

	template <class Msg>
	bool loadAll(const std::string& channel, std::vector<Msg>& msg)
	{
		static_assert(ros::message_traits::IsMessage<Msg>::value, "Default load only defined for ROS message types.");
		std::lock_guard<std::mutex> lock(mtx);
		rosbag::View view(*bag, TopicTypeQuery(channel, ros::message_traits::DataType<Msg>::value()));

		// There may be many valid messages matching the request; we return the first one we find.
		for (const rosbag::MessageInstance& m : view)
		{
			auto p = m.instantiate<Msg>();
			if (p)
			{
				msg.push_back(*p);
			}
		}
		return true;
	}

	template <class Msg>
	bool loadAll(const std::string& channel, std::vector<boost::shared_ptr<Msg>>& msg)
	{
		static_assert(ros::message_traits::IsMessage<Msg>::value, "Default load only defined for ROS message types.");
		std::lock_guard<std::mutex> lock(mtx);
		rosbag::View view(*bag, TopicTypeQuery(channel, ros::message_traits::DataType<Msg>::value()));

		// There may be many valid messages matching the request; we return the first one we find.
		for (const rosbag::MessageInstance& m : view)
		{
			auto p = m.instantiate<Msg>();
			if (p)
			{
				msg.push_back(p);
			}
		}
		return true;
	}

	std::string getDefaultPath() const;
	ros::Time getTime() const;

	static std::unique_ptr<DataLog> instance;
};

template <>
void DataLog::log<const std::string&>(const std::string& channel, const std::string& msg);

template <>
std::string DataLog::load<std::string>(const std::string& channel);

template <>
void DataLog::log<const std::vector<char>>(const std::string& channel, const std::vector<char>& msg);

}

#endif // SRC_DATALOGGER_H
