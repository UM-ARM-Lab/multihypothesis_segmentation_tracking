//
// Created by arprice on 8/13/18.
//

#include <opencv2/imgproc.hpp>
#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <cudaSift/image.h>
#include <cudaSift/sift.h>
#include <cudaSift/utils.h>

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

const size_t MAX_BUFFER_LEN = 100;//1000;

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
	cv::Mat gray1, gray2, tempGray, desc1, desc2;

	double fps = MAX_BUFFER_LEN/(rgb_buffer.back()->header.stamp - rgb_buffer.front()->header.stamp).toSec();
	cv::VideoWriter video("source.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer.front()->image.size(), true);
	cv::VideoWriter tracking("tracking.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer.front()->image.size(), true);

//	auto detector = cv::AKAZE::create();
//	auto detector = cv::KAZE::create();
//	auto detector = cv::xfeatures2d::SIFT::create();

	const unsigned int w = rgb_buffer.front()->image.cols;
	const unsigned int h = rgb_buffer.front()->image.rows;

	const float initBlur = 1.0f;
	const float thresh = 2.5f;
	const float pixel_motion_thresh = 200;

	cudaSift::SiftData siftData1, siftData2;
	const int maxPts = 10000; // 32768
	cudaSift::InitSiftData(siftData1, maxPts, true, true);
	cudaSift::InitSiftData(siftData2, maxPts, true, true);

	siftData1.stream = 0;// if both are 0 they just operate on the same stream,
	siftData2.stream = 0;// AKA if not using streams this can be skipped

	cudaSift::InitCuda(0);
	cudaSift::Image img1, img2;

//	rgb_buffer.front()->image.convertTo(gray1, CV_32FC1);
	cv::cvtColor(rgb_buffer.front()->image, tempGray, cv::COLOR_BGR2GRAY);
	tempGray.convertTo(gray1, CV_32FC1);
	img1.Allocate(w, h, cudaSift::iAlignUp(w, 128), false, nullptr, (float*)gray1.data);
	img1.stream = 0;
	img1.Download();
	cudaSift::ExtractSift(siftData1, img1, 5, initBlur, thresh, 0.0f, false);

//	cv::cvtColor(rgb_buffer.front()->image, gray1, cv::COLOR_BGR2GRAY);
//	detector->detectAndCompute(gray1, cv::noArray(), kpts1, desc1);


	for (int i = 1; i < static_cast<int>(rgb_buffer.size()) && ros::ok(); ++i)
	{
		rgb_buffer[i-1]->image.copyTo(display);

		cv::cvtColor(rgb_buffer[i]->image, tempGray, cv::COLOR_BGR2GRAY);
		tempGray.convertTo(gray2, CV_32FC1);
		img2.Allocate(w, h, cudaSift::iAlignUp(w, 128), false, nullptr, (float*)gray2.data);
		img2.stream = 0;
		img2.Download();
		cudaSift::ExtractSift(siftData2, img2, 5, initBlur, thresh, 0.0f, false);

//		cv::cvtColor(rgb_buffer[i]->image, gray2, cv::COLOR_BGR2GRAY);
//		detector->detectAndCompute(gray2, cv::noArray(), kpts2, desc2);

		cudaSift::MatchSiftData(siftData1, siftData2);

		for (int p = 0; p < siftData1.numPts; ++p)
		{
			const cudaSift::SiftPoint& sp = siftData1.h_data[p];
			assert(sp.match >= 0);
			assert(sp.match < siftData2.numPts);
			const cudaSift::SiftPoint& spm = siftData2.h_data[sp.match];
			{
				if (sqrt(pow(sp.xpos-spm.xpos,2)+pow(sp.ypos-spm.ypos,2)) > pixel_motion_thresh) { continue; }
				cv::circle(display, cv::Point2f(sp.xpos, sp.ypos), sp.scale, cv::Scalar(255, 0, 0));
				cv::circle(display, cv::Point2f(spm.xpos, spm.ypos), spm.scale, cv::Scalar(0, 255, 0));
				cv::line(display, cv::Point2f(sp.xpos, sp.ypos), cv::Point2f(spm.xpos, spm.ypos), cv::Scalar(0, 0, 255));
			}
		}

		video.write(rgb_buffer[i-1]->image);
		tracking.write(display);
//		cv::imshow("prev", display);
//		cv::waitKey(1);

		std::swap(gray1, gray2);
//		std::swap(kpts1, kpts2);
//		std::swap(desc1, desc2);
		std::swap(img1, img2);
		std::swap(siftData1, siftData2);

		img2.Destroy();
	}

	cudaSift::FreeSiftData(siftData1);
	cudaSift::FreeSiftData(siftData2);

	video.write(rgb_buffer.back()->image);
	video.release();
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
	cv::namedWindow("a", cv::WINDOW_GUI_NORMAL);
	cv::namedWindow("b", cv::WINDOW_GUI_NORMAL);

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