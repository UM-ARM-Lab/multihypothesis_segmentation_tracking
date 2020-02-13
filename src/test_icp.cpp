//
// Created by kunhuang on 2/7/20.
//
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>

#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <opencv2/highgui.hpp>
#include <pcl_ros/point_cloud.h> // Needed to publish a point cloud

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

void test_icp()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);

	// Fill in the CloudIn data
	cloud_in->width    = 5;
	cloud_in->height   = 1;
	cloud_in->is_dense = false;
	cloud_in->points.resize (cloud_in->width * cloud_in->height);
	for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
	{
		cloud_in->points[i].x = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].y = 1024 * rand () / (RAND_MAX + 1.0f);
		cloud_in->points[i].z = 1024 * rand () / (RAND_MAX + 1.0f);
	}
	std::cout << "Saved " << cloud_in->points.size () << " data points to input:"
	          << std::endl;
	for (std::size_t i = 0; i < cloud_in->points.size (); ++i) std::cout << "    " <<
	                                                                     cloud_in->points[i].x << " " << cloud_in->points[i].y << " " <<
	                                                                     cloud_in->points[i].z << std::endl;
	*cloud_out = *cloud_in;
	std::cout << "size:" << cloud_out->points.size() << std::endl;
	for (std::size_t i = 0; i < cloud_in->points.size (); ++i)
		cloud_out->points[i].x = cloud_in->points[i].x + 0.7f;
	std::cout << "Transformed " << cloud_in->points.size () << " data points:"
	          << std::endl;
	for (std::size_t i = 0; i < cloud_out->points.size (); ++i)
		std::cout << "    " << cloud_out->points[i].x << " " <<
		          cloud_out->points[i].y << " " << cloud_out->points[i].z << std::endl;

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);
	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);
	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
	          icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_icp");
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

	std::unique_ptr<objectActionModel> oam = std::make_unique<objectActionModel>(10);
	std::unique_ptr<Tracker> sparseTracker = std::make_unique<Tracker>();
	sparseTracker->track_options.featureRadius = 200.0f;
	sparseTracker->track_options.pixelRadius = 1000.0f;
	sparseTracker->track_options.meterRadius = 1.0f;
	std::unique_ptr<DenseTracker> denseTracker = std::make_unique<SiamTracker>();

	cv::Mat temp_seg = seg_out.objectness_segmentation->image;
	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup = getBBox(temp_seg, seg_out.roi);

	for (auto& pair : labelToBBoxLookup)
	{
		oam->sampleAction(buffer_out, seg_out, sparseTracker, denseTracker, pair.first, pair.second);

		std::cout << "-------------------------------------------------------------------------------------" << std::endl;

		/////////////////////////////////////////////
		//// Construct tracking time steps
		/////////////////////////////////////////////
		std::vector<ros::Time> steps; // SiamMask tracks all these time steps except the first frame;
		for (auto iter = buffer_out.rgb.begin(); iter != buffer_out.rgb.end(); std::advance(iter, 5))
		{
			steps.push_back(iter->first);
		}

		/////////////////////////////////////////////
		//// Look up worldTcamera
		/////////////////////////////////////////////
		const std::string tableFrame = "table_surface";
		tf::StampedTransform worldTcameraTF;
		geometry_msgs::TransformStamped wTc = buffer_out.tfs->lookupTransform(tableFrame, buffer_out.cameraModel.tfFrame(), ros::Time(0));
		tf::transformStampedMsgToTF(wTc, worldTcameraTF);
		moveit::Pose worldTcamera;
		tf::transformTFToEigen(worldTcameraTF, worldTcamera);

		/////////////////////////////////////////////
		//// SiamMask tracking
		/////////////////////////////////////////////
		std::map<ros::Time, cv::Mat> masks;
		denseTracker->track(steps, buffer_out, pair.second, masks);

		//// Fill in the first frame mask
		cv::Mat startMask = cv::Mat::zeros(buffer_out.rgb.begin()->second->image.size(), CV_8UC1);
		cv::Mat subwindow(startMask, seg_out.roi);
		subwindow = pair.first == seg_out.objectness_segmentation->image;
		masks.insert(masks.begin(), {steps.front(), startMask});

		//// Construct PointCloud Segments
		pcl::PointCloud<PointT>::Ptr initCloudSegment = make_PC_segment(buffer_out.rgb[steps[0]]->image, buffer_out.depth[steps[0]]->image,
		                                                                buffer_out.cameraModel, masks[steps[0]]);
		if (initCloudSegment->empty()) std::cerr << "Init cloud segment is empty!" << std::endl;
		pcl::PointCloud<PointT>::Ptr lastCloudSegment = make_PC_segment(buffer_out.rgb[steps[ steps.size()-1 ]]->image,
		                                                                buffer_out.depth[steps[ steps.size()-1 ]]->image,
		                                                                buffer_out.cameraModel,
		                                                                masks[steps[ steps.size()-1 ]]);
		if (lastCloudSegment->empty()) std::cerr << "Last cloud segment is empty!" << std::endl;

//		ros::Publisher pcPub = nh.advertise<pcl::PointCloud<PointT>>("segment_clouds", 1, true);
//		initCloudSegment->header.frame_id = buffer_out.cameraModel.tfFrame();
//		pcl_conversions::toPCL(ros::Time::now(), scene->cloud->header.stamp);
//		pcPub.publish(*initCloudSegment);
//		std::cerr << "cloud." << std::endl;
//		sleep(3);

		//// ICP
		pcl::IterativeClosestPoint<PointT, PointT> icp;
		icp.setInputSource(initCloudSegment);
		icp.setInputTarget(lastCloudSegment);
		pcl::PointCloud<PointT> Final;
		icp.align(Final);
		std::cout << "has converged:" << icp.hasConverged() << " score: " <<
		          icp.getFitnessScore() << std::endl;
		Eigen::Matrix<float, 4, 4> Mcamera = icp.getFinalTransformation();
		Eigen::Matrix<double, 4, 4> Mworld = worldTcamera.matrix() * Mcamera.cast<double>() * worldTcamera.inverse().matrix();
		std::cout << Mworld << std::endl;

		rigidTF twist;
		twist.linear = {Mworld(0, 3), Mworld(1, 3), Mworld(2, 3)};
		double theta = acos((Mworld(0, 0) + Mworld(1, 1) + Mworld(2, 2) - 1) / 2.0);
		if (theta == 0) twist.angular = {0, 0, 0};
		else
		{
			Eigen::Vector3d temp = {Mworld(2, 1)-Mworld(1, 2), Mworld(0, 2)-Mworld(2, 0), Mworld(1, 0)-Mworld(0,1)};
			Eigen::Vector3d omega = 1/(2 * sin(theta)) * temp;
			twist.angular = theta * omega;
		}
		std::cerr << "ICP TF: ";
		std::cerr << "\t Linear: " << twist.linear.x() << " " << twist.linear.y() << " " << twist.linear.z();
		std::cerr << "\t Angular: " << twist.angular.x() << " " << twist.angular.y() << " " << twist.angular.z() << std::endl;
	}

//	test_icp();
	return 0;
}
