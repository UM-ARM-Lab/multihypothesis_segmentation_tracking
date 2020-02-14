//
// Created by kunhuang on 2/4/20.
//

#include "mps_voxels/logging/log_cv_roi.h"
#include "mps_voxels/util/macros.h"

#include <sensor_msgs/RegionOfInterest.h>

namespace mps
{

template <>
void DataLog::log<cv::Rect>(const std::string& channel, const cv::Rect& msg)
{
	activeChannels.insert(channel + "/roi"); \
	sensor_msgs::RegionOfInterest out_msg;
	out_msg.x_offset = msg.x;
	out_msg.y_offset = msg.y;
	out_msg.height = msg.height;
	out_msg.width = msg.width;
	log(channel + "/roi", out_msg);
}

template <>
bool DataLog::load<cv::Rect>(const std::string& channel, cv::Rect& msg)
{
	sensor_msgs::RegionOfInterest out_msg;
	load(channel + "/roi" , out_msg);
	msg.x = out_msg.x_offset;
	msg.y = out_msg.y_offset;
	msg.height = out_msg.height;
	msg.width = out_msg.width;
	return true;
}

}
