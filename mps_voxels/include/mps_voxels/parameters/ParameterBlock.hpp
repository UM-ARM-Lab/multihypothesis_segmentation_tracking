//
// Created by pricear on 2020-03-26.
//

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
