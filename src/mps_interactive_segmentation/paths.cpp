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

#include "mps_interactive_segmentation/paths.h"

#include <gazebo/common/SystemPaths.hh>

#include <ros/package.h>

#include <boost/algorithm/string.hpp>
#include <boost/filesystem.hpp>

#include <fstream>
#include <sstream>

namespace mps
{

const std::string PACKAGE_PREFIX = "package://";
const std::string MODEL_PREFIX = "model://";

std::string parsePackageURL(const std::string& url)
{
	size_t is_package = url.find(PACKAGE_PREFIX);
	if (std::string::npos == is_package)
	{
		// Not a package path
		return url;
	}

	std::string filename = url;
	filename.erase(0, PACKAGE_PREFIX.length());
	size_t pos = filename.find('/');
	if (pos != std::string::npos)
	{
		std::string package = filename.substr(0, pos);
		filename.erase(0, pos);
		std::string package_path = ros::package::getPath(package);
		if (package_path.empty())
		{
			throw std::runtime_error("Could not find package '" + package + "'.");
		}
		filename = package_path + filename;
	}

	return filename;
}

std::string parseModelURL(const std::string& url)
{
	return gazebo::common::SystemPaths::Instance()->FindFileURI(url);
}

// From: https://stackoverflow.com/questions/4891006/how-to-create-a-folder-in-the-home-directory
std::string expandUser(std::string path)
{
	if (not path.empty() and path[0] == '~')
	{
		assert(path.size() == 1 or path[1] == '/');  // or other error handling
		char const* home = getenv("HOME");
		if (home or ((home = getenv("USERPROFILE"))))
		{
			path.replace(0, 1, home);
		}
		else
		{
			char const *hdrive = getenv("HOMEDRIVE"),
				*hpath = getenv("HOMEPATH");
			assert(hdrive);  // or other error handling
			assert(hpath);
			path.replace(0, 1, std::string(hdrive) + hpath);
		}
	}
	return path;
}

namespace fs = boost::filesystem;
std::vector<std::string> getGazeboPaths()
{
//	std::vector<std::string> paths;
//
//	// Check for GAZEBO_MODEL_PATH
//	const std::string sep = ":";
//	char const* env_path = getenv("GAZEBO_MODEL_PATH");
//	if (env_path)
//	{
//		boost::algorithm::split(paths, env_path, boost::is_any_of(sep));
//	}
//	else
//	{
//		paths = {"~/.gazebo"};
//	}
//
//	for (auto& path : paths)
//	{
//		path = expandUser(path);
//		path = fs::canonical(path).string();
//	}
//
//
//
//	// Check for ~/.gazebo
//	return paths;
	auto P = gazebo::common::SystemPaths::Instance()->GetGazeboPaths();
	return {P.begin(), P.end()};
}

}
