//
// Created by kunhuang on 9/24/19.
//

#include "mps_voxels/SiamTracker.h"

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

void SiamTracker::track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const cv::Mat& initMask, std::map<ros::Time, cv::Mat>& masks)
{
	cv::Rect box = cv::boundingRect(initMask);

	mps_msgs::AABBox2d bbox;
	bbox.xmin = box.x;
	bbox.xmax = box.x + box.width;
	bbox.ymin = box.y;
	bbox.ymax = box.y + box.height;

	track(steps, buffer, bbox, masks);
}

void SiamTracker::track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const mps_msgs::AABBox2d& initRegion, std::map<ros::Time, cv::Mat>& masks)
{

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	actionClient.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	mps_msgs::TrackBBoxGoal goal;
	goal.bbox = initRegion;

	if (buffer.rgb.empty() || buffer.depth.empty())
	{
		ROS_WARN_STREAM("Tracking failed: Capture buffer empty.");
		return;
	}
	int numframes = static_cast<int>(steps.size());
	std::cerr << "Number of time steps: " << numframes << std::endl;
//	sensor_msgs::Image video[numframes];
	for (int i = 0; i < numframes && ros::ok(); i++) // TODO: do NOT change i-step here; otherwise, masks don't match with vector<Time> steps
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
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
	{
		ROS_ERROR("Action did not finish before the time out.");
		return;
	}

	// each raw tracking result takes around 300Mb at 0.5fps, too large!!!
	const auto& res = actionClient.getResult();


	if (steps.size() !=  res->mask.size() + 1)
	{
		ROS_ERROR_STREAM("SiamMask did not return the correct number of frames!!!");
		return;
	}

	std::vector<cv::Mat> ims;
//	std::vector<std::vector<std::vector<bool>>> masks;
	std::cerr << "Number of frames returned from SiamMask: " << res->mask.size() << std::endl;
//	for (auto iter = actionClient.getResult()->mask.begin(); iter != actionClient.getResult()->mask.end(); iter++).
	for (size_t i = 0; i < res->mask.size(); ++i)
	{
		// only store the first frame and the last frame:
//		if (iter != actionClient.getResult()->mask.begin() && iter != actionClient.getResult()->mask.end()-1) continue;

//		std::vector<std::vector<bool>> maskBool;
//		maskBool.resize(iter->height, std::vector<bool>(iter->width));

//		auto temp = iter->data;
		const sensor_msgs::Image& im = res->mask[i];
//		std::cerr << "Tracking result type: " << iter->encoding << std::endl;
		cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(im, im.encoding);
		cv::Mat mask = cv_ptr->image > 0; // uchar
		masks.insert(masks.end(), {steps[i+1], mask});

//		cv::imwrite("/home/kunhuang/Pictures/mask.jpg", im);
//		cv::waitKey(0);
//		for (size_t i = 0; i < iter->height; i++)
//		{
//			for (size_t j = 0; j < iter->width; j++)
//			{
//				maskBool[i][j] = (int) im.at<uint8_t>(i, j) > 0;
//			}
//		}
//		ims.push_back(im);
//		masks.push_back(maskBool);
	}

//	labelToTrackingLookup.insert({label, ims});
//	labelToMasksLookup.insert({label, masks});
}

/*
void SiamTracker::siamtrack(LabelT label, const std::vector<ros::Time>& steps, mps_msgs::AABBox2d bbox, const SensorHistoryBuffer& buffer)
{
	actionlib::SimpleActionClient<mps_msgs::TrackBBoxAction> ac("TrackBBox", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	mps_msgs::TrackBBoxGoal goal;
	goal.bbox = bbox;

	if (buffer.rgb.empty() || buffer.depth.empty())
	{
		ROS_WARN_STREAM("Tracking failed: Capture buffer empty.");
		return;
	}
	int numframes = static_cast<int>(steps.size());
//	sensor_msgs::Image video[numframes];
	for (int i = 0; i < numframes && ros::ok(); ++i)
	{
		cv::Mat im = buffer.rgb.at(steps[i])->image;
		cv_bridge::CvImage out_msg;
//		out_msg.header   = im.header; // Same timestamp and tf frame as input image
		out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
		out_msg.image = im; // Your cv::Mat

		goal.video.push_back(*out_msg.toImageMsg());
	}

	ac.sendGoal(goal);

	//wait for the action to return
	bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = ac.getState();
		ROS_INFO("Action finished: %s", state.toString().c_str());
	}
	else
		ROS_INFO("Action did not finish before the time out.");

	std::vector<cv::Mat> ims;
	for (auto iter = ac.getResult()->mask.begin(); iter != ac.getResult()->mask.end(); iter++)
	{
//		std::cerr << iter->encoding << std::endl;
		cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(*iter, iter->encoding);
		cv::Mat im = cv_ptr->image;
		ims.push_back(im);
	}

	labelToTrackingLookup.insert({label, ims});
}*/

}