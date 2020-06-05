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

#ifndef MPS_PARAMETERBLOCK_HPP
#define MPS_PARAMETERBLOCK_HPP

#include <ros/node_handle.h>
#include <yaml-cpp/yaml.h>

#include <string>
#include <exception>

namespace mps
{

template <typename ProviderType>
struct ParameterBlock
{
//	template <typename T>
//	bool get(const std::string& path, T& );
};


template <typename T, typename ProviderType>
T getParam(const ProviderType& p, const std::string& key)
{
	return ParameterBlock<ProviderType>::template get<T>(p, key);
}

template <typename T, typename ProviderType>
T getParam(const ProviderType& p, const char* key)
{
	return getParam<T, ProviderType>(p, std::string(key));
}

template <>
struct ParameterBlock<ros::NodeHandle>
{
	using ProviderType = ros::NodeHandle;

	template <typename T>
	static
	T get(const ProviderType& p, const std::string& key)
	{
		T retVal;
		bool gotParam = p.getParam(key, retVal);
		if (!gotParam) { throw std::runtime_error("Failed to look up parameter '" + key + "'"); }
		return retVal;
	}
};

template <>
struct ParameterBlock<YAML::Node>
{
	using ProviderType = YAML::Node;

	template <typename T>
	static
	T get(const ProviderType& p, const std::string& key)
	{
		const auto idx = key.find_first_of('/');
		if (std::string::npos != idx)
		{
			const auto parentKey = key.substr(0, idx);
			const auto childKey = key.substr(idx + 1);
			if (!p[parentKey])
			{
				throw std::logic_error("Key '" + parentKey + "' not found.");
			}
			return get<T>(p[parentKey], childKey);
		}
		else
		{
			if (!p[key])
			{
				// TODO: Better debugging by reporting the full path
				throw std::logic_error("Key '" + key + "' not found.");
			}
			return p[key].as<T>();
		}
	}
};



}
#endif //MPS_PARAMETERBLOCK_HPP
