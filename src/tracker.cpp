//
// Created by arprice on 8/13/18.
//

//#include <opencv2/imgproc.hpp>
//#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
//
//#include <cudaSift/image.h>
//#include <cudaSift/sift.h>
//#include <cudaSift/utils.h>
//
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>
//#include <depth_image_proc/depth_conversions.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

#include "mps_voxels/CudaTracker.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/segmentation_utils.h"

#include <visualization_msgs/MarkerArray.h>
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>

#include <ucm2hier.hpp>

#include <ros/ros.h>

std::unique_ptr<Tracker> tracker;
std::shared_ptr<RGBDSegmenter> segmentationClient;
std::unique_ptr<image_transport::Publisher> segmentationPub;

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


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "flow");
	ros::NodeHandle nh, pnh("~");

//	image_transport::ImageTransport it(nh);
//	image_transport::TransportHints hints("compressed", ros::TransportHints(), pnh);
	image_transport::ImageTransport it(nh);

	segmentationClient = std::make_shared<RGBDSegmenter>(nh);
	segmentationPub = std::make_unique<image_transport::Publisher>(it.advertise("segmentation", 1));

	cv::namedWindow("segmentation", CV_WINDOW_NORMAL);
	cv::namedWindow("contours", CV_WINDOW_NORMAL);

	tracker = std::make_unique<CudaTracker>(50);

	while (ros::ok())
	{
		if (tracker->rgb_buffer.size() == tracker->MAX_BUFFER_LEN)
		{
			int step = 3;
			tracker->track(1);
			for (size_t i = 0; i < tracker->MAX_BUFFER_LEN; i+=step)
			{
				cv::Rect roi(500, 500,
				             1000, 500);
				int pyrScale = 4;
				cv::Mat rgb_cropped(tracker->rgb_buffer[i]->image, roi); cv::pyrDown(rgb_cropped, rgb_cropped);// cv::pyrDown(rgb_cropped, rgb_cropped);
				cv::Mat depth_cropped(tracker->depth_buffer[i]->image, roi); cv::pyrDown(depth_cropped, depth_cropped);// cv::pyrDown(depth_cropped, depth_cropped);
				cv_bridge::CvImage cv_rgb_cropped(tracker->rgb_buffer[i]->header, tracker->rgb_buffer[i]->encoding, rgb_cropped);
				cv_bridge::CvImage cv_depth_cropped(tracker->depth_buffer[i]->header, tracker->depth_buffer[i]->encoding, depth_cropped);
				sensor_msgs::CameraInfo cam_msg_cropped = tracker->cameraModel.cameraInfo();
				cam_msg_cropped.width = roi.width/pyrScale;
				cam_msg_cropped.height = roi.height/pyrScale;
				cam_msg_cropped.K[2] -= roi.x/pyrScale;
				cam_msg_cropped.K[5] -= roi.y/pyrScale;
				for (auto& d : cam_msg_cropped.D) { d = 0.0; }

				cv_bridge::CvImagePtr contours;
				cv_bridge::CvImagePtr seg = segmentationClient->segment(cv_rgb_cropped, cv_depth_cropped, cam_msg_cropped, &contours);
				if (!seg)
				{
					ROS_ERROR_STREAM("Segmentation failed.");
					break;
				}

				double alpha = 0.75;
				cv::Mat labelColorsMap = colorByLabel(seg->image);
				labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*rgb_cropped;
				cv::imshow("segmentation", labelColorsMap);
				cv::waitKey(1);

				cv::Mat tempContours1, tempContours2;
				double maxVal;
				cv::minMaxLoc(contours->image, nullptr, &maxVal);
				contours->image.convertTo(tempContours1, CV_8UC1, 255.0/maxVal);
				cv::applyColorMap(tempContours1, tempContours2, cv::COLORMAP_JET); // COLORMAP_HOT
				cv::imshow("contours", tempContours2);
				cv::waitKey(1);

				if (segmentationPub->getNumSubscribers() > 0)
				{
					segmentationPub->publish(cv_bridge::CvImage(tracker->cameraModel.cameraInfo().header, "bgr8", labelColorsMap).toImageMsg());
				}

				Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> contour_map(contours->image.ptr<double>(), contours->image.rows, contours->image.cols);
				Eigen::Map<Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> label_map(seg->image.ptr<uint16_t>(), seg->image.rows, seg->image.cols);
				merging_sequence tree = ucm2hier(contour_map, label_map.cast<label_type>());
				tree.print();
				std::cerr << tree.start_ths.size() << std::endl;
			}
			tracker->reset();
		}
		cv::waitKey(1);
		ros::Duration(1.0).sleep();
		cv::waitKey(1);
	}

	return 0;
}