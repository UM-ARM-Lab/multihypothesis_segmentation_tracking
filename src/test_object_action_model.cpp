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
#include "mps_voxels/ObjectActionModel.h"

#include <mps_msgs/ClusterRigidMotionsAction.h>

#include <octomap/octomap.h>
#include <unordered_set>
#include <opencv2/highgui.hpp>
#include <tf_conversions/tf_eigen.h>

#include <ros/ros.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

using namespace mps;

void test_track()
{
	/////////////////////////////////////////////
	//// Load sensor history and segInfo
	/////////////////////////////////////////////
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
		DataLog loader("/home/kunhuang/mps_log/test_segInfo_" + worldname + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("segInfo");
		loader.load<SegmentationInfo>("segInfo", seg_out);
		std::cerr << "Successfully loaded." << std::endl;
	}
	std::cerr << "roi in loaded segInfo: " << seg_out.roi.x << " " << seg_out.roi.y << " " << seg_out.roi.height << " " << seg_out.roi.width << std::endl;


	/////////////////////////////////////////////
	//// Construct tracking time steps
	/////////////////////////////////////////////
	std::vector<ros::Time> steps; // SiamMask tracks all these time steps except the first frame;
	for (auto iter = buffer_out.rgb.begin(); iter != buffer_out.rgb.end(); std::advance(iter, 5))
	{
		steps.push_back(iter->first);
	}
	std::vector<ros::Time> timeStartEnd;
	timeStartEnd.push_back(steps[0]);
	timeStartEnd.push_back(steps[steps.size()-1]);

	cv::Mat temp_seg = seg_out.objectness_segmentation->image;

	std::unique_ptr<Tracker> sparseTracker;
	std::unique_ptr<DenseTracker> denseTracker = std::make_unique<SiamTracker>();
	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup = getBBox(temp_seg, seg_out.roi);

	sparseTracker = std::make_unique<Tracker>();
	sparseTracker->track_options.featureRadius = 400.0f;
	sparseTracker->track_options.pixelRadius = 100.0f;
	sparseTracker->track_options.meterRadius = 1.0f;

	/////////////////////////////////////////////
	//// Look up worldTcamera
	/////////////////////////////////////////////
	const std::string tableFrame = "table_surface";
	tf::StampedTransform worldTcameraTF;
	geometry_msgs::TransformStamped wTc = buffer_out.tfs->lookupTransform(tableFrame, buffer_out.cameraModel.tfFrame(), ros::Time(0));
	tf::transformStampedMsgToTF(wTc, worldTcameraTF);
	moveit::Pose worldTcamera;
	tf::transformTFToEigen(worldTcameraTF, worldTcamera);

	std::unique_ptr<objectActionModel> oam = std::make_unique<objectActionModel>();

	for (const auto& pair : labelToBBoxLookup)
	{
		std::cout << "-------------------------------------------------------------------------------------" << std::endl;
		//// SiamMask tracking
		std::map<ros::Time, cv::Mat> masks;
		denseTracker->track(steps, buffer_out, pair.second, masks);

		//// Fill in the first frame mask
		cv::Mat startMask = cv::Mat::zeros(buffer_out.rgb.begin()->second->image.size(), CV_8UC1);
		cv::Mat subwindow(startMask, seg_out.roi);
		subwindow = pair.first == seg_out.objectness_segmentation->image;
		masks.insert(masks.begin(), {steps.front(), startMask});

		//// Estimate motion using SiamMask
		Eigen::Vector3d roughMotion = oam->sampleActionFromMask(masks[steps[0]], buffer_out.depth[steps[0]]->image,
		                                                   masks[steps[steps.size()-1]], buffer_out.depth[steps[steps.size()-1]]->image,
		                                                   buffer_out.cameraModel, worldTcamera);
		std::cerr << "Rough Motion from SiamMask: " << roughMotion.x() << " " << roughMotion.y() << " " << roughMotion.z() << std::endl;

		//// SIFT
		sparseTracker->track(timeStartEnd, buffer_out, masks, "/home/kunhuang/Videos/" + std::to_string((int)pair.first) + "_");

		/////////////////////////////////////////////
		//// send request to jlinkage server
		/////////////////////////////////////////////
		oam->clusterRigidBodyTransformation(sparseTracker->flows3, worldTcamera);
	}


	/////////////////////////////////////////////
	//// sample object motions
	/////////////////////////////////////////////


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

	return 0;
}