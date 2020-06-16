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

#ifndef MPS_EXPERIMENT_H
#define MPS_EXPERIMENT_H

#include <ros/ros.h>
#include <ros/service_client.h>

#include <string>
#include <map>
#include <random>

namespace mps
{

class Experiment
{
public:
	Experiment(ros::NodeHandle& nh, ros::NodeHandle& pnh);

	const ros::WallTime experiment_start;
	std::string experiment_id;
	std::string experiment_dir;
	ros::ServiceClient externalVideoClient;

	bool visualizeAll = false;
	std::map<std::string, bool> visualize;
	inline
	bool shouldVisualize(const std::string& channel) const
	{
		if (visualizeAll) { return true; }

		const auto& iter = visualize.find(channel);
		return (iter != visualize.end() && iter->second);
	}

	std::string timestamp() const;

//protected:

	mutable
	std::default_random_engine rng; // TODO: Should this be mutex-protected?

};

}

#endif // MPS_EXPERIMENT_H
