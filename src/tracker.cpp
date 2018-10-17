//
// Created by arprice on 8/13/18.
//

#include "mps_voxels/Tracker.h"

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include "mps_voxels/image_utils.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/colormap.h"

#include <opencv2/highgui.hpp>
#include <ros/ros.h>

std::shared_ptr<RGBDSegmenter> segmentationClient;
std::unique_ptr<Tracker> tracker;
std::unique_ptr<image_transport::Publisher> segmentationPub;

void track(Tracker& t)
{

	for (size_t k = 0; k < t.rgb_buffer.size(); ++k)
	{
		cv_bridge::CvImagePtr cv_rgb_ptr = t.rgb_buffer[k];
		cv_bridge::CvImagePtr cv_depth_ptr = t.depth_buffer[k];


		cv_bridge::CvImagePtr seg = segmentationClient->segment(*cv_rgb_ptr, *cv_depth_ptr, t.cameraModel.cameraInfo());
		if (!seg)
		{
			ROS_ERROR_STREAM("Segmentation failed.");
			return;
		}
		double alpha = 0.75;
		cv::Mat labelColorsMap = colorByLabel(seg->image);
		labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*cv_rgb_ptr->image;

		if (segmentationPub->getNumSubscribers() > 0)
		{
			segmentationPub->publish(cv_bridge::CvImage(t.cameraModel.cameraInfo().header, "bgr8", labelColorsMap).toImageMsg());
		}

	}
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "flow");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);

	segmentationClient = std::make_shared<RGBDSegmenter>(nh);
	segmentationPub = std::make_unique<image_transport::Publisher>(it.advertise("segmentation", 1));

	Tracker::TrackingOptions track_options;
	std::string topic_prefix = "/kinect2_victor_head/hd";
	Tracker::SubscriptionOptions sub_options(topic_prefix);

	const int track_frame_count = 10;
	Tracker t(track_frame_count, sub_options, track_options);
	t.startCapture();
	for (int i = 0; i < 60; ++i)
	{
		ros::Duration(1.0).sleep();
		if (track_frame_count == static_cast<int>(t.rgb_buffer.size()))
		{
			break;
		}
	}
	t.stopCapture();

//	track(t);
	t.track();

	cv::waitKey(0);
	ros::spin();

	return 0;
}