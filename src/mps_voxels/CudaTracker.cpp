//
// Created by arprice on 10/18/18.
//

#ifdef HAS_CUDA_SIFT

#include "mps_voxels/CudaTracker.h"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <cudaSift/image.h>
#include <cudaSift/sift.h>
#include <cudaSift/utils.h>

#include <opencv2/highgui.hpp>

CudaTracker::CudaTracker(tf::TransformListener* _listener,
                         const size_t _buffer,
                         SubscriptionOptions _options,
                         TrackingOptions _track_options)
                         : Tracker(_listener, _buffer, std::move(_options), std::move(_track_options))
{

}

void CudaTracker::track(const std::vector<ros::Time>& steps)
{
	if (rgb_buffer.empty() || depth_buffer.empty())
	{
		ROS_WARN_STREAM("Tracking failed: Capture buffer empty.");
		return;
	}

	mask = getMask();

	////////////////////////////////////////
	//// Set up Keypoint tracking
	////////////////////////////////////////

	cv::Mat display;
	cv::Mat gray1, gray2, tempGray;

	const ros::Time& tFirst = steps.front();
	const ros::Time& tLast = steps.back();

	double fps = steps.size()/(tLast - tFirst).toSec();
	cv::VideoWriter video("source.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer[tFirst]->image.size(), true);
	cv::VideoWriter tracking("tracking.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer[tFirst]->image.size(), true);

	const unsigned int w = rgb_buffer[tFirst]->image.cols;
	const unsigned int h = rgb_buffer[tFirst]->image.rows;

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

	// Convert to 32-bit grayscale and compute SIFT
	cv::cvtColor(rgb_buffer.begin()->second->image, tempGray, cv::COLOR_BGR2GRAY);
	tempGray.convertTo(gray1, CV_32FC1);
	img1.Allocate(w, h, cudaSift::iAlignUp(w, 128), false, nullptr, (float*)gray1.data);
	img1.stream = 0;
	img1.Download();
	cudaSift::ExtractSift(siftData1, img1, 5, initBlur, thresh, 0.0f, false);

	for (int i = 1; i < static_cast<int>(steps.size()) && ros::ok(); ++i)
	{
		const ros::Time& tPrev = steps[i-1];
		const ros::Time& tCurr = steps[i];

		rgb_buffer[tPrev]->image.copyTo(display);

		// Convert to 32-bit grayscale and compute SIFT
		cv::cvtColor(rgb_buffer[tCurr]->image, tempGray, cv::COLOR_BGR2GRAY);
		tempGray.convertTo(gray2, CV_32FC1);
		img2.Allocate(w, h, cudaSift::iAlignUp(w, 128), false, nullptr, (float*)gray2.data);
		img2.stream = 0;
		img2.Download();
		cudaSift::ExtractSift(siftData2, img2, 5, initBlur, thresh, 0.0f, false);

		cudaSift::MatchSiftData(siftData1, siftData2);

		Flow2D flow2;
		Flow3D flow3;

		for (int p = 0; p < siftData1.numPts; ++p)
		{
			const cudaSift::SiftPoint& sp = siftData1.h_data[p];
			assert(sp.match >= 0);
			assert(sp.match < siftData2.numPts);
			const cudaSift::SiftPoint& spm = siftData2.h_data[sp.match];
			{
				// Enforce max pixel motion constraint
				if (sqrt(pow(sp.xpos-spm.xpos,2)+pow(sp.ypos-spm.ypos,2)) > pixel_motion_thresh) { continue; }

				cv::Point2i pt1(sp.xpos, sp.ypos);
				cv::Point2i pt2(spm.xpos, spm.ypos);

				// Check mask
				if (!mask.at<uint8_t>(pt1) || !mask.at<uint8_t>(pt2))
				{
					continue;
				}

				// Get depths and validity
				uint16_t dVal1 = depth_buffer[tPrev]->image.at<uint16_t>(pt1);
				uint16_t dVal2 = depth_buffer[tCurr]->image.at<uint16_t>(pt2);
				if (!(DepthTraits::valid(dVal1) && DepthTraits::valid(dVal2))) { continue; }

				// Get 3D motion vector
				float depth1 = DepthTraits::toMeters(dVal1); // if (depth1 > maxZ || depth1 < minZ) { continue; }
				float depth2 = DepthTraits::toMeters(dVal2); // if (depth2 > maxZ || depth2 < minZ) { continue; }
				Vector p1 = toPoint3D<Vector>(sp.xpos, sp.ypos, depth1, cameraModel);
				Vector p2 = toPoint3D<Vector>(spm.xpos, spm.ypos, depth2, cameraModel);
				Vector v = p2-p1;
				const double dist_thresh = track_options.meterRadius;
				if ((v.x()*v.x() + v.y()*v.y() + v.z()*v.z()) > dist_thresh*dist_thresh) { continue; }

				// Draw the match
				cv::circle(display, cv::Point2f(sp.xpos, sp.ypos), sp.scale, cv::Scalar(255, 0, 0));
				cv::circle(display, cv::Point2f(spm.xpos, spm.ypos), spm.scale, cv::Scalar(0, 255, 0));
				cv::line(display, cv::Point2f(sp.xpos, sp.ypos), cv::Point2f(spm.xpos, spm.ypos), cv::Scalar(0, 0, 255));

				flow2.push_back({pt1, pt2});
				flow3.push_back({p1, v});
			}
		}

		visualization_msgs::MarkerArray ma;
		ma.markers.push_back(visualizeFlow(flow3));
		vizPub.publish(ma);

		flows2[{tPrev, tCurr}] = flow2;
		flows3[{tPrev, tCurr}] = flow3;

		video.write(rgb_buffer[tPrev]->image);
		tracking.write(display);
//		cv::imshow("prev", display);
//		cv::waitKey(1);

		std::swap(gray1, gray2);
		std::swap(img1, img2);
		std::swap(siftData1, siftData2);

		img2.Destroy();
	}

	cudaSift::FreeSiftData(siftData1);
	cudaSift::FreeSiftData(siftData2);

	video.write(rgb_buffer[tLast]->image);
	video.release();
	tracking.release();
}

#endif //#ifdef HAS_CUDA_SIFT
