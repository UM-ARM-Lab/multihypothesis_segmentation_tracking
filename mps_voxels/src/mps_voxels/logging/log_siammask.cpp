//
// Created by kunhuang on 6/10/20.
//

#include "mps_voxels/logging/log_siammask.h"
#include "mps_voxels/logging/log_cv_mat.h"
#include <std_msgs/Header.h>

namespace mps
{
using t2mask = std::map<ros::Time, cv::Mat>;

template <>
void DataLog::log<SiamMaskData>(const std::string& channel, const SiamMaskData& msg)
{
	std_msgs::Header header;
//	header.frame_id = cameraModel.tfFrame();

	for (auto& pair : msg)
	{
		std::string subchannel = channel + "/" + std::to_string(pair.first);
		activeChannels.insert(subchannel);
		for (const auto& m : pair.second)
		{
			header.stamp = m.first;
			log(subchannel, toMaskMessage(m.second, header));
		}

	}
}

template <>
SiamMaskData DataLog::load<SiamMaskData>(const std::string& channel)
{
	SiamMaskData msg;
	for (auto& c : activeChannels)
	{
		LabelT label = (LabelT)std::stoi(c.substr(channel.size()+1));
		t2mask temp;

		std::vector<sensor_msgs::CompressedImage> masks;
		loadAll(c, masks);
		for (const auto& m : masks)
		{
			cv::Mat cvMask = fromCompressedMessage(m);
			temp.emplace(m.header.stamp, cvMask);
		}

		msg.emplace(label, temp);
	}

	return msg;
}
}
