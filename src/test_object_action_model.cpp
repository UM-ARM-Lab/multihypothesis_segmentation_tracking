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

#include <mps_msgs/ClusterRigidMotionsAction.h>

#include <octomap/octomap.h>
#include <unordered_set>
#include <opencv2/highgui.hpp>

#include <ros/ros.h>
#include <actionlib/client/terminal_state.h>
#include <actionlib/client/simple_action_client.h>

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
		DataLog loader("/home/kunhuang/mps_log/test_segInfo_" + worldname + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("segInfo");
		loader.load<SegmentationInfo>("segInfo", seg_out);
		std::cerr << "Successfully loaded." << std::endl;
	}
	std::cerr << "roi in loaded segInfo: " << seg_out.roi.x << " " << seg_out.roi.y << " " << seg_out.roi.height << " " << seg_out.roi.width << std::endl;


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

//#ifdef USE_CUDA_SIFT
//	sparseTracker = std::make_unique<CudaTracker>();
//#else
	sparseTracker = std::make_unique<Tracker>();
	sparseTracker->track_options.featureRadius = 400.0f;
	sparseTracker->track_options.pixelRadius = 100.0f;
	sparseTracker->track_options.meterRadius = 1.0f;
//#endif

	actionlib::SimpleActionClient<mps_msgs::ClusterRigidMotionsAction> jlinkageActionClient("cluster_flow", true);

	for (const auto& pair : labelToBBoxLookup)
	{
		//// SiamMask tracking
		std::map<ros::Time, cv::Mat> masks;
		denseTracker->track(steps, buffer_out, pair.second, masks);

		//// Sift on SimaMask result
		cv::Mat startMask = cv::Mat::zeros(buffer_out.rgb.begin()->second->image.size(), CV_8UC1);
		cv::Mat subwindow(startMask, seg_out.roi);
		subwindow = pair.first == seg_out.objectness_segmentation->image;
		masks.insert(masks.begin(), {steps.front(), startMask});
		sparseTracker->track(timeStartEnd, buffer_out, masks, "/home/kunhuang/Videos/" + std::to_string((int)pair.first) + "_");

		/////////////////////////////////////////////
		//// send request to jlinkage server
		/////////////////////////////////////////////

		// TODO: matlab action server
		for (auto& t2f : sparseTracker->flows3) // go through all time steps
		{
			mps_msgs::ClusterRigidMotionsGoal goal;
			mps_msgs::ClusterRigidMotionsResultConstPtr response;
			for (auto& f : t2f.second)
			{
				mps_msgs::FlowVector fv;
				fv.pos.x = f.first.x();
				fv.pos.y = f.first.y();
				fv.pos.z = f.first.z();
				fv.vel.x = f.second.x();
				fv.vel.y = f.second.y();
				fv.vel.z = f.second.z();
				goal.flow_field.push_back(fv);
			}
			if (!jlinkageActionClient.isServerConnected())
			{
				std::cerr << "jlinkage server not connected" << std::endl;
			}

			auto success = jlinkageActionClient.sendGoalAndWait(goal);
			if (!success.isDone())
			{
				std::cerr << "jlinkage not done" << std::endl;
			}

			const auto& res = jlinkageActionClient.getResult();
			for (auto& t : res->motions)
			{
				std::cerr << "Linear: " << t.linear.x << " " << t.linear.y << " " << t.linear.z << std::endl;
				std::cerr << "Angular: " << t.angular.x << " " << t.angular.y << " " << t.angular.z << std::endl;
			}

		}
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