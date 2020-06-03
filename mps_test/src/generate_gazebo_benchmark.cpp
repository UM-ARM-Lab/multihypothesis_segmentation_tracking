/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mps_voxels/util/package_paths.h>
#include <mps_voxels/SensorHistorian.h>
#include <mps_voxels/segmentation_utils.h>
#include <mps_voxels/image_utils.h>
#include <mps_voxels/logging/DataLog.h>
#include <mps_voxels/logging/log_segmentation_info.h>
#include <mps_voxels/logging/log_sensor_history.h>
#include <mps_voxels/logging/log_cv_mat.h>

#include <ros/ros.h>

#include <boost/filesystem.hpp>

namespace fs = boost::filesystem;
using namespace mps;

bool generateSensorHistory(SensorHistoryBuffer& buff)
{
	SensorHistorian::SubscriptionOptions opts;
	opts.hints = image_transport::TransportHints();
	auto historian = std::make_unique<SensorHistorian>(500, opts);
	historian->startCapture();
	for (int attempt = 0; attempt < 1000; ++attempt)
	{
		ros::Duration(0.1).sleep();
		if (!historian->buffer.rgb.empty()) { break; }
	}
	historian->stopCapture();
	if (historian->buffer.rgb.empty())
	{
		ROS_ERROR_STREAM("Failed to get any data.");
		return false;
	}

	buff = historian->buffer;

	return true;
}

std::shared_ptr<SegmentationInfo>
segment(const cv_bridge::CvImage& rgb, const cv_bridge::CvImage& depth, const sensor_msgs::CameraInfo& cam)
{
//	using LabelType = uint16_t;
	using SegmentationClient = actionlib::SimpleActionClient<mps_msgs::SegmentRGBDAction>;

	ros::NodeHandle nh;
	SegmentationClient segmentClient(nh, "/segment_rgbd", true);
	if (!segmentClient.waitForServer(ros::Duration(30)))
	{
		ROS_ERROR("Segmentation server not connected.");
		return {};
	}

	mps_msgs::SegmentRGBDGoal request;
	mps_msgs::SegmentRGBDResultConstPtr response;

	rgb.toImageMsg(request.rgb);
	depth.toImageMsg(request.depth);
	request.camera_info = cam;

	if (!segmentClient.isServerConnected())
	{
		ROS_ERROR("Segmentation server not connected.");
		return std::shared_ptr<SegmentationInfo>();
	}

	auto success = segmentClient.sendGoalAndWait(request);
	if (!success.isDone())
	{
		ROS_ERROR("Segmentation server did not report finished.");
		return std::shared_ptr<SegmentationInfo>();
	}

	response = segmentClient.getResult();
	if (response->segmentation.data.empty())
	{
		ROS_ERROR("Segmentation results empty.");
		return std::shared_ptr<SegmentationInfo>();
	}

	std::shared_ptr<SegmentationInfo> si = std::make_shared<SegmentationInfo>();
	si->t = cam.header.stamp;
	si->frame_id = cam.header.frame_id;
	si->rgb = rgb.image;
	si->depth = depth.image;
	si->roi.width = cam.roi.width;
	si->roi.height = cam.roi.height;
	si->roi.x = cam.roi.x_offset;
	si->roi.y = cam.roi.y_offset;

	si->objectness_segmentation = cv_bridge::toCvCopy(response->segmentation, "mono16");

	return si;
}

bool generateSegmentationInfo(SensorHistoryBuffer& buff, SegmentationInfo& info)
{
	bool res = generateSensorHistory(buff);
	if (!res)
	{
		ROS_ERROR_STREAM("Failed to get sensor history.");
		return false;
	}

	auto si = segment(*buff.rgb.begin()->second, *buff.depth.begin()->second, buff.cameraModel.cameraInfo());
	if (!si)
	{
		ROS_ERROR_STREAM("Failed to get segmentation.");
		return false;
	}

	info = *si;
	return true;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "generate_gazebo_benchmark");
	ros::NodeHandle pnh("~");

	std::string worldFileParam;
	if (!pnh.getParam("/simulation_world", worldFileParam))
	{
		ROS_FATAL_STREAM("Unable to read world file name from parameter server '" << worldFileParam << "'");
		return -1;
	}
	// Load Gazebo world file and get meshes
	const std::string gazeboWorldFilename = parsePackageURL(worldFileParam);
	const std::string worldName = fs::path(gazeboWorldFilename).stem().string();

	// Check for existence of mps_test_data
	const std::string testDirName = "package://mps_test_data/";
	if (!(fs::is_directory(parsePackageURL(testDirName)) && fs::exists(parsePackageURL(testDirName))))
	{
		throw std::runtime_error("Unable to find test data directory.");
	}

	std::string bagFileName = parsePackageURL(testDirName) + "/" + worldName + ".bag";

	// Wait for gazebo to start up and publish /clock
	while (!ros::Time::isValid())
	{
		sleep(1);
	}
	sleep(10);

	SensorHistoryBuffer buff;
	SegmentationInfo info;

	bool generated = generateSegmentationInfo(buff, info);
	if (!generated)
	{
		ROS_FATAL_STREAM("Unable to get ground truth segmentation!");
		return -1;
	}

	cv::Mat labels = info.objectness_segmentation->image;
	cv::Mat labelColors = colorByLabel(labels);
	labelColors.setTo(0, 0 == labels);

	std::shared_ptr<DataLog> log = std::make_shared<DataLog>(bagFileName, std::unordered_set<std::string>{"sensor_history", "ground_truth/labels", "ground_truth/visual"}, rosbag::BagMode::Write);
	log->log("sensor_history", buff);
	log->log("ground_truth/labels", *info.objectness_segmentation->toImageMsg());
	log->log("ground_truth/visual", toMessage(labelColors, info.objectness_segmentation->header));

	return 0;
}