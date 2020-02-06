//
// Created by kunhuang on 2/4/20.
//

#include "mps_voxels/VoxelSegmentation.h"
#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_cv_roi.h"
#include "mps_voxels/logging/log_sensor_history.h"
#include "mps_voxels/logging/log_segmentation_info.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/Tracker.h"
#include "mps_voxels/SiamTracker.h"

#include <octomap/octomap.h>
#include <unordered_set>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <actionlib/client/terminal_state.h>

using namespace mps;

void test_track()
{
	std::string worldname = "experiment_world";
	SensorHistoryBuffer buffer_out;
	{
		DataLog loader("/home/kunhuang/mps_log/explorer_buffer_" + worldname + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("buffer");
		loader.load<SensorHistoryBuffer>("buffer", buffer_out);
		std::cerr << "Successfully loaded." << std::endl;
	}
	std::cerr << "number of frames: " << buffer_out.rgb.size() << std::endl;

	SegmentationInfo seg_out;
	{
		DataLog loader("/home/kunhuang/mps_log/explorer_segInfo_" + worldname + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("segInfo");
		loader.load<SegmentationInfo>("segInfo", seg_out);
		std::cerr << "Successfully loaded." << std::endl;
	}

	cv::Rect roi_out;
	{
		DataLog loader("/home/kunhuang/mps_log/explorer_roi_" + worldname + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("roi");
		loader.load<cv::Rect>("roi", roi_out);
		std::cerr << "Successfully loaded." << std::endl;
	}

	// TODO: Fix this!
	seg_out.roi = roi_out;
	std::cerr << "roi: " << roi_out.x << " " << roi_out.y << " " << roi_out.height << " " << roi_out.width << std::endl;

	std::vector<ros::Time> steps; // SiamMask tracks all these time steps except the first frame;
	for (auto iter = buffer_out.rgb.begin(); iter != buffer_out.rgb.end(); std::advance(iter, 5))
	{
		steps.push_back(iter->first);
	}

	cv::Mat temp_seg = seg_out.objectness_segmentation->image;

	std::unique_ptr<Tracker> sparseTracker;
	std::unique_ptr<DenseTracker> denseTracker = std::make_unique<SiamTracker>();
	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup = getBBox(temp_seg, roi_out);

//#ifdef USE_CUDA_SIFT
//	sparseTracker = std::make_unique<CudaTracker>();
//#else
	sparseTracker = std::make_unique<Tracker>();
//#endif

	for (const auto& pair : labelToBBoxLookup)
	{
		std::map<ros::Time, cv::Mat> masks;
		denseTracker->track(steps, buffer_out, pair.second, masks);
		cv::Mat startMask = cv::Mat::zeros(buffer_out.rgb.begin()->second->image.size(), CV_8UC1);
		cv::Mat subwindow(startMask, seg_out.roi);
		subwindow = pair.first == seg_out.objectness_segmentation->image;
//		cv::imwrite("/home/kunhuang/silly.jpg", startMask); //cv::cvtColor(buffer.rgb.at(tCurr)->image, gray2, cv::COLOR_BGR2GRAY);
		masks.insert(masks.begin(), {steps.front(), startMask});
		sparseTracker->track(steps, buffer_out, masks, "/home/kunhuang/Videos/");
	}


	/////////////////////////////////////////////
	//// sample object motions
	/////////////////////////////////////////////
//	for (auto pair:tracker->labelToMasksLookup)
//	{
//		std::cerr << "mask size = " << pair.second.size() << " x " << pair.second[0].size() << " x "
//		          << pair.second[0][0].size() << std::endl;
//
//		//// SIFT
//		tracker->siftOnMask(steps, buffer_out, pair.first);
//		std::cerr << "SIFT completed!!!" << std::endl;
//	}

}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_track");
	ros::NodeHandle nh;
	if (!nh.hasParam("/use_sim_time"))
	{
		ROS_INFO("No param named '/use_sim_time'");
	}

	nh.setParam("/use_sim_time", false);
	test_track();

	//exit
	return 0;
}