//
// Created by kunhuang on 2/4/20.
//

#include "mps_voxels/logging/log_cv_roi.h"
#include "mps_voxels/util/macros.h"

namespace mps
{

sensor_msgs::RegionOfInterest
ros_message_conversion<cv::Rect>::toMessage(const cv::Rect& t, const std::nullptr_t)
{
	sensor_msgs::RegionOfInterest out_msg;
	out_msg.x_offset = t.x;
	out_msg.y_offset = t.y;
	out_msg.height = t.height;
	out_msg.width = t.width;
	return out_msg;
}

cv::Rect
ros_message_conversion<cv::Rect>::fromMessage(const sensor_msgs::RegionOfInterest& t)
{
	cv::Rect msg;
	msg.x = t.x_offset;
	msg.y = t.y_offset;
	msg.height = t.height;
	msg.width = t.width;
	return msg;
}

template <>
void DataLog::log<cv::Rect>(const std::string& channel, const cv::Rect& msg)
{
	activeChannels.insert(channel + "/roi"); \
	log(channel + "/roi", toMessage(msg));
}

template <>
cv::Rect DataLog::load<cv::Rect>(const std::string& channel)
{
	auto out_msg = load<sensor_msgs::RegionOfInterest>(channel + "/roi");
	return fromMessage(out_msg);
}

}
