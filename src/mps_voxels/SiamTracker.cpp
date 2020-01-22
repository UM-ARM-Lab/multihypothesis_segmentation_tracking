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

namespace mps
{

SiamTracker::SiamTracker(TrackingOptions _track_options)
	:Tracker(std::move(_track_options))
{

}

void SiamTracker::track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, LabelT label)
{
	actionlib::SimpleActionClient<mps_msgs::TrackBBoxAction> ac("TrackBBox", true);

	ROS_INFO("Waiting for action server to start.");
	// wait for the action server to start
	ac.waitForServer(); //will wait for infinite time

	ROS_INFO("Action server started, sending goal.");
	// send a goal to the action
	mps_msgs::TrackBBoxGoal goal;
	goal.bbox = labelToBBoxLookup[label];

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