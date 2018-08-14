//
// Created by arprice on 8/13/18.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <depth_image_proc/depth_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>

#include <visualization_msgs/MarkerArray.h>
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>

#include <ros/ros.h>

const size_t MAX_BUFFER_LEN = 200;//1000;

std::vector<cv_bridge::CvImagePtr> rgb_buffer;
std::vector<cv_bridge::CvImagePtr> depth_buffer;
sensor_msgs::CameraInfo cam;

static void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,
                           double, const cv::Scalar& color)
{
	for(int y = 0; y < cflowmap.rows; y += step)
		for(int x = 0; x < cflowmap.cols; x += step)
		{
			const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
			cv::line(cflowmap, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
			     color);
			cv::circle(cflowmap, cv::Point(x,y), 2, color, -1);
		}
}
/*
void track()
{
	cv::Mat flow, cflow, display;
	cv::UMat gray1, gray2, uflow; // Possibly CPU or GPU image matrix
	cv::cvtColor(rgb_buffer.front()->image, gray1, cv::COLOR_BGR2GRAY);
	for (int i = 1; i < static_cast<int>(rgb_buffer.size()) && ros::ok(); ++i)
	{
		rgb_buffer[i-1]->image.copyTo(display);
		cv::cvtColor(rgb_buffer[i]->image, gray2, cv::COLOR_BGR2GRAY);

//		cv::calcOpticalFlowFarneback(gray1, gray2, uflow, 0.4, 3, 12, 5, 7, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
//		cv::calcOpticalFlowFarneback(gray1, gray2, uflow, 0.4, 1, 12, 5, 7, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
//		uflow.copyTo(flow);
		cv::calcOpticalFlowFarneback(gray1, gray2, uflow, 0.3, 4, 5, 15, 5, 1.2, cv::OPTFLOW_FARNEBACK_GAUSSIAN);

		const int GRID_STEP = 15;

		cv::cvtColor(gray1, cflow, cv::COLOR_GRAY2BGR);
		uflow.copyTo(flow);
		drawOptFlowMap(flow, cflow, 16, 1.5, cv::Scalar(0, 255, 0));

//		// By y += 5, x += 5 you can specify the grid
//		for (int y = 0; y < display.rows; y += GRID_STEP)
//		{
//			for (int x = 0; x < display.cols; x += GRID_STEP)
//			{
//				// get the flow from y, x position * 10 for better visibility
//				const cv::Point2f flowatxy = flow.at<cv::Point2f>(y, x) * 10;
//				// draw line at flow direction
//				cv::line(display, cv::Point(x, y), cv::Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), cv::Scalar(255,0,0));
//				// draw initial point
//				cv::circle(display, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1);
//			}
//		}

//		cv::imshow("prev", display);
		cv::imshow("prev", cflow);
		cv::waitKey(100);

		std::swap(gray1, gray2);
	}
}
*/

void track()
{
	cv::Mat display;
	std::vector<cv::KeyPoint> kpts1, kpts2;
	cv::UMat gray1, gray2, desc1, desc2;

	double fps = MAX_BUFFER_LEN/(rgb_buffer.back()->header.stamp - rgb_buffer.front()->header.stamp).toSec();
	cv::VideoWriter video("source.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer.front()->image.size(), true);
	cv::VideoWriter tracking("tracking.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer.front()->image.size(), true);

//	auto detector = cv::AKAZE::create();
//	auto detector = cv::KAZE::create();
	auto detector = cv::xfeatures2d::SIFT::create();

	cv::cvtColor(rgb_buffer.front()->image, gray1, cv::COLOR_BGR2GRAY);
	detector->detectAndCompute(gray1, cv::noArray(), kpts1, desc1);
	for (int i = 1; i < static_cast<int>(rgb_buffer.size()) && ros::ok(); ++i)
	{
		rgb_buffer[i-1]->image.copyTo(display);
		cv::cvtColor(rgb_buffer[i]->image, gray2, cv::COLOR_BGR2GRAY);

		detector->detectAndCompute(gray2, cv::noArray(), kpts2, desc2);

		cv::BFMatcher matcher(cv::NORM_L2);//(cv::NORM_HAMMING);
		std::vector< std::vector<cv::DMatch> > nn_matches;
//		matcher.knnMatch(desc1, desc2, nn_matches, 3);
		matcher.radiusMatch(desc1, desc2, nn_matches, 450.0);

		for (int ii = 0; ii < static_cast<int>(nn_matches.size()); ++ii)
		{
			nn_matches[ii].erase(std::remove_if(nn_matches[ii].begin(), nn_matches[ii].end(),
			                                   [&](const cv::DMatch& match){
				                                   cv::Point diff = kpts2[match.trainIdx].pt - kpts1[match.queryIdx].pt;
				                                   return std::sqrt(diff.x*diff.x + diff.y*diff.y) > 20.0;
			                                   }),
			                    nn_matches[ii].end());

			for (int jj = 0; jj < static_cast<int>(nn_matches[ii].size()); ++jj)
			{
				const cv::DMatch& match = nn_matches[ii][jj];
				cv::circle(display, kpts1[match.queryIdx].pt, kpts1[match.queryIdx].size, cv::Scalar(255, 0, 0));
				cv::circle(display, kpts2[match.trainIdx].pt, kpts2[match.trainIdx].size, cv::Scalar(0, 255, 0));
				cv::line(display, kpts1[match.queryIdx].pt, kpts2[match.trainIdx].pt, cv::Scalar(0, 0, 255));
			}
		}

//		cv::drawKeypoints(display, kpts1, display, cv::Scalar(255, 0, 0));
//		cv::drawKeypoints(display, kpts2, display, cv::Scalar(0, 255, 0));

		video.write(rgb_buffer[i-1]->image);
		tracking.write(display);
		cv::imshow("prev", display);
		cv::waitKey(1);

		std::swap(gray1, gray2);
		std::swap(kpts1, kpts2);
		std::swap(desc1, desc2);
	}

	video.write(rgb_buffer.back()->image);
	tracking.release();
}

void imageCb(const sensor_msgs::ImageConstPtr& rgb_msg,
             const sensor_msgs::ImageConstPtr& depth_msg,
             const sensor_msgs::CameraInfo& cam_msg)
{
	cv_bridge::CvImagePtr cv_rgb_ptr;
	try
	{
		cv_rgb_ptr = cv_bridge::toCvCopy(rgb_msg, sensor_msgs::image_encodings::BGR8);
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	cv_bridge::CvImagePtr cv_depth_ptr;
	try
	{
		cv_depth_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1); // MONO16?
	}
	catch (cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	if (rgb_buffer.size() < MAX_BUFFER_LEN)
	{
		rgb_buffer.push_back(cv_rgb_ptr);
		std::cerr << rgb_buffer.size() << ": " << rgb_buffer.back()->header.stamp - rgb_buffer.front()->header.stamp << std::endl;
	}

	if (depth_buffer.size() < MAX_BUFFER_LEN)
	{
		depth_buffer.push_back(cv_depth_ptr);
	}

	if (rgb_buffer.size() == MAX_BUFFER_LEN)
	{
		track();
		ros::shutdown();
	}

	cam = cam_msg;
}

using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "flow");
	ros::NodeHandle nh, pnh("~");

	cv::namedWindow("prev", cv::WINDOW_GUI_NORMAL);

	image_transport::ImageTransport it(nh);
	image_transport::TransportHints hints("compressed", ros::TransportHints(), pnh);

	const int buffer = 10;

	std::string topic_prefix = "/kinect2_victor_head/hd";
	image_transport::SubscriberFilter rgb_sub(it, topic_prefix+"/image_color_rect", buffer, hints);
	image_transport::SubscriberFilter depth_sub(it, topic_prefix+"/image_depth_rect", buffer, hints);
	message_filters::Subscriber<sensor_msgs::CameraInfo> info_sub(nh, topic_prefix+"/camera_info", buffer);

	message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(buffer), rgb_sub, depth_sub, info_sub);
	sync.registerCallback(imageCb);

//	image_flow_pub = std::make_unique<image_transport::Publisher>(it.advertise("/kinect2_victor_head/qhd/image_flow", 1));
//	vecPub = nh.advertise<visualization_msgs::Marker>("scene_flow", buffer, true);

//	ros::Subscriber sub = nh.subscribe ("kinect2_roof/qhd/points", 1, cloud_cb);

	ros::spin();

	return 0;
}