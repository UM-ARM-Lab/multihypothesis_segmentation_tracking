//
// Created by kunhuang on 9/24/19.
//

#include "mps_voxels/SiamTracker.h"
#include "mps_voxels/logging/log_siammask.h"

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <mps_msgs/TrackBBoxAction.h>
#include <mps_msgs/AABBox2d.h>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui.hpp>

namespace mps
{

SiamTracker::SiamTracker()
	: DenseTracker(), actionClient("TrackBBox", true)
{

}

bool SiamTracker::track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, uint16_t label, const cv::Mat& initMask, std::map<ros::Time, cv::Mat>& masks)
{
	cv::Rect box = cv::boundingRect(initMask);

	mps_msgs::AABBox2d bbox;
	bbox.xmin = box.x;
	bbox.xmax = box.x + box.width;
	bbox.ymin = box.y;
	bbox.ymax = box.y + box.height;

	return track(steps, buffer, label, bbox, masks);
}

bool SiamTracker::track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, uint16_t /*label*/, const mps_msgs::AABBox2d& initRegion, std::map<ros::Time, cv::Mat>& masks)
{

	ROS_INFO("Waiting for SiamMask server to start.");
	// wait for the action server to start
	actionClient.waitForServer(); //will wait for infinite time

	ROS_INFO("SiamMask server started, sending goal.");
	// send a goal to the action
	mps_msgs::TrackBBoxGoal goal;
	goal.bbox = initRegion;

	if (buffer.rgb.empty() || buffer.depth.empty())
	{
		ROS_WARN_STREAM("Tracking failed: Capture buffer empty.");
		return false;
	}
	int numframes = static_cast<int>(steps.size());
//	std::cerr << "Number of time steps: " << numframes << std::endl;
//	sensor_msgs::Image video[numframes];
	for (int i = 0; i < numframes && ros::ok(); i++) //// do NOT change i-step here; otherwise, masks don't match with vector<Time> steps
	{
		cv::Mat im = buffer.rgb.at(steps[i])->image;
		cv_bridge::CvImage out_msg;
//		out_msg.header   = im.header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
		out_msg.image = im; // Your cv::Mat

		goal.video.push_back(*out_msg.toImageMsg());
	}
	std::cerr << "Number of frames sent to SiamMask: " << goal.video.size() << std::endl;

	actionClient.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = actionClient.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionClient.getState();
		ROS_INFO("SiamMask action finished: %s", state.toString().c_str());
	}
	else
	{
		ROS_ERROR("SiamMask action did not finish before the time out.");
		return false;
	}

	const auto& serverStateForGoal = actionClient.getState();
	if (serverStateForGoal != actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		ROS_ERROR_STREAM("SiamMask action reported state '" << serverStateForGoal.toString() << "': " << serverStateForGoal.getText());
		if (serverStateForGoal == actionlib::SimpleClientGoalState::RECALLED)
		{
			ROS_ERROR_STREAM("Goal was recalled. Did time go backwards?");
		}
		return false;
	}

	// each raw tracking result takes around 300Mb at 0.5fps, too large!!!
	const auto& res = actionClient.getResult();

	if (steps.size() !=  res->mask.size() + 1)
	{
		ROS_ERROR_STREAM("SiamMask only returns the " << res->mask.size() << " frames!!!");
//		return;
	}

	std::vector<cv::Mat> ims;
	for (size_t i = 0; i < res->mask.size(); ++i)
	{
		// only store the first frame and the last frame:
//		if (iter != actionClient.getResult()->mask.begin() && iter != actionClient.getResult()->mask.end()-1) continue;

		const sensor_msgs::CompressedImage& im = res->mask[i];
		cv::Mat extracted = cv::imdecode(im.data, cv::IMREAD_GRAYSCALE);
		cv::Mat mask = extracted > 0; // uchar
		masks.insert(masks.end(), {steps[i+1], mask});
	}
	return true;
}

bool SiamTracker::isHistoryTracker(std::string& /*fname*/)
{
	return false;
}

HistoryTracker::HistoryTracker(const std::string &path)
	: logger(path, {}, rosbag::bagmode::Read), trackingFilename(path)
{

}

bool HistoryTracker::track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, uint16_t label, const cv::Mat& /*initMask*/, std::map<ros::Time, cv::Mat>& masks)
{
	mps_msgs::AABBox2d bbox;
	return track(steps, buffer, label, bbox, masks);
}

bool HistoryTracker::track(const std::vector<ros::Time>& /*steps*/, const SensorHistoryBuffer& /*buffer*/, uint16_t label,
                           const mps_msgs::AABBox2d& /*initRegion*/, std::map<ros::Time, cv::Mat>& masks)
{
	logger.activeChannels.insert("SiamMaskData/" + std::to_string(label));
	SiamMaskData siam_out = logger.load<SiamMaskData>("SiamMaskData");
	if (siam_out.empty()) return false;
	masks = siam_out[label];
	return true;
}

bool HistoryTracker::isHistoryTracker(std::string& fname)
{
	fname = trackingFilename;
	return true;
}

}