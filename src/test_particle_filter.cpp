//
// Created by kunhuang on 2/12/20.
//

#include <iostream>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/highgui.hpp>

#include "mps_voxels/ParticleFilter.h"
#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_sensor_history.h"
#include "mps_voxels/logging/log_segmentation_info.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/pointcloud_utils.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/Tracker.h"
#include "mps_voxels/SiamTracker.h"
#include "mps_voxels/ObjectActionModel.h"

using namespace mps;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_PF");
	ros::NodeHandle nh;
	if (!nh.hasParam("/use_sim_time"))
	{
		ROS_INFO("No param named '/use_sim_time'");
	}
	nh.setParam("/use_sim_time", false);

	/////////////////////////////////////////////
	//// Load sensor history and segInfo
	/////////////////////////////////////////////
	std::string worldname = "singleBeer_02_07";
//	std::string worldname = "experiment_world_02_07";
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
	std::cerr << "roi in loaded segInfo: " << seg_out.roi.x << " " << seg_out.roi.y << " " << seg_out.roi.height << " " << seg_out.roi.width << std::endl;


	/////////////////////////////////////////////
	//// Particle Filter
	/////////////////////////////////////////////
	std::unique_ptr<ParticleFilter> pf = std::make_unique<ParticleFilter>(2);
	std::unique_ptr<objectActionModel> oam = std::make_unique<objectActionModel>(1);
	std::unique_ptr<DenseTracker> denseTracker = std::make_unique<SiamTracker>();
	std::unique_ptr<Tracker> sparseTracker = std::make_unique<Tracker>();
	sparseTracker->track_options.featureRadius = 200.0f;
	sparseTracker->track_options.pixelRadius = 1000.0f;
	sparseTracker->track_options.meterRadius = 1.0f;

	cv::Mat temp_seg = seg_out.objectness_segmentation->image;
	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup = getBBox(temp_seg, seg_out.roi);

	for (auto& pair : labelToBBoxLookup)
	{
		oam->sampleAction(buffer_out, seg_out, sparseTracker, denseTracker, pair.first, pair.second);
	}

	return 0;
}
