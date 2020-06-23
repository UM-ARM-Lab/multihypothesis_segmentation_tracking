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

#include "mps_voxels/Experiment.h"

#include <boost/uuid/random_generator.hpp>
#include <boost/uuid/uuid_io.hpp>

#include <boost/format.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace mps
{


Experiment::Experiment(ros::NodeHandle& nh, ros::NodeHandle& pnh)
	: experiment_start(ros::WallTime::now())
{
	std::random_device rd;

	const std::string ns = "/experiment";

	bool gotParam = false;
	const std::string idKey = ns + "/id";
	gotParam = nh.getParam(idKey, experiment_id);
//	if (!gotParam)
	{
		boost::mt19937 ran(rd());
		boost::uuids::basic_random_generator<boost::mt19937> gen(ran);
		boost::uuids::uuid u = gen();
		experiment_id = boost::uuids::to_string(u);
		nh.setParam(idKey, experiment_id);
	}
	ROS_INFO_STREAM("Experiment ID: " << experiment_id);

	const std::string dirKey = ns + "/directory";
	gotParam = nh.getParam(dirKey, experiment_dir);
//	if (!gotParam)
	{
		fs::path temp_dir = fs::temp_directory_path();
		fs::path working_dir = temp_dir /
		                       fs::path(ros::this_node::getName()) /
		                       fs::path(timestamp()); // /
//		                       fs::path(experiment_id);
		fs::create_directories(working_dir);
		experiment_dir = working_dir.string();
		nh.setParam(dirKey, experiment_dir);
	}
	ROS_INFO_STREAM("Experiment directory: " << experiment_dir);

	std::vector<std::string> visualizeChannels;
	pnh.param("visualize", visualizeChannels, std::vector<std::string>());
//	if (!visualizeChannels.empty())
	{
		ROS_INFO_STREAM("Visualizing channels:");
		for (auto& m : visualizeChannels)
		{
			std::transform(m.begin(), m.end(), m.begin(), ::tolower);
			if (m == "all") { visualizeAll = true; }
			ROS_INFO_STREAM("\t" << m);
			visualize.emplace(m, true);
		}
	}

	const std::string seedKey = ns + "/seed";
	int seed = 0;
	gotParam = pnh.getParam(seedKey, seed);
//	if (!gotParam)
	{
		seed = rd();
		nh.setParam(seedKey, seed);
	}
	ROS_INFO_STREAM("Random Seed: " << seed);
	rng = std::mt19937(seed);

	const std::string timeKey = ns + "/start_time";
	nh.setParam(timeKey, timestamp());

}

std::string Experiment::timestamp() const
{
	boost::posix_time::ptime p_time = experiment_start.toBoost();
	return boost::posix_time::to_iso_extended_string(p_time);
}

}
