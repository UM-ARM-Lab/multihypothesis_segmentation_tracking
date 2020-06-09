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

#ifdef HAS_CUDA_SIFT

#include "mps_voxels/CudaTracker.h"
#include "mps_voxels/project_point.hpp"

#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <cudaSift/image.h>
#include <cudaSift/sift.h>
#include <cudaSift/utils.h>

#include <opencv2/highgui.hpp>

namespace mps
{

CudaTracker::CudaTracker(TrackingOptions _track_options)
	: Tracker(std::move(_track_options))
{

}

void CudaTracker::track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const std::map<ros::Time, cv::Mat>& masks)
{
	if (buffer.rgb.empty() || buffer.depth.empty())
	{
		ROS_WARN_STREAM("Tracking failed: Capture buffer empty.");
		return;
	}

	mask = getMask(buffer);

	////////////////////////////////////////
	//// Set up Keypoint tracking
	////////////////////////////////////////

	cv::Mat display;
	cv::Mat gray1, gray2, tempGray;

	const ros::Time& tFirst = steps.front();
	const ros::Time& tLast = steps.back();

	cv::Mat maskPrev;
	if (!masks.empty()) { maskPrev = mask & masks.at(tFirst); }
	else { maskPrev = mask; }


	std::unique_ptr<cv::VideoWriter> video, tracking;
	bool shouldRecord = !track_options.directory.empty();
	if (shouldRecord)
	{
		auto format = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
		auto size = buffer.rgb.at(tFirst)->image.size();
		double fps = std::max(1.0, steps.size() / (tLast - tFirst).toSec());
		video = std::make_unique<cv::VideoWriter>(track_options.directory + "/source.avi", format, fps, size, true);
		tracking = std::make_unique<cv::VideoWriter>(track_options.directory + "/tracking.avi", format, fps, size, true);
	}

	const unsigned int w = buffer.rgb.at(tFirst)->image.cols;
	const unsigned int h = buffer.rgb.at(tFirst)->image.rows;

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
	cv::cvtColor(buffer.rgb.begin()->second->image, tempGray, cv::COLOR_BGR2GRAY);
	tempGray.convertTo(gray1, CV_32FC1);
	img1.Allocate(w, h, cudaSift::iAlignUp(w, 128), false, nullptr, (float*) gray1.data);
	img1.stream = 0;
	img1.Download();
	cudaSift::ExtractSift(siftData1, img1, 5, initBlur, thresh, 0.0f, false);

	// TODO: Apply mask before performing matching
//	for (int p = 0; p < siftData1.numPts; ++p)
//	{
//		const cudaSift::SiftPoint& sp = siftData1.h_data[p];
//		const cv::Point2i pt1(sp.xpos, sp.ypos);
//		if (!maskPrev.at<uint8_t>(pt1))
//		{
//
//		}
//	}

	for (int i = 1; i < static_cast<int>(steps.size()) && ros::ok(); ++i)
	{
		const ros::Time& tPrev = steps[i - 1];
		const ros::Time& tCurr = steps[i];

		buffer.rgb.at(tPrev)->image.copyTo(display);

		cv::Mat maskCurr;
		if (!masks.empty()) { maskCurr = mask & masks.at(tCurr); }
		else { maskCurr = mask; }

		// Convert to 32-bit grayscale and compute SIFT
		cv::cvtColor(buffer.rgb.at(tCurr)->image, tempGray, cv::COLOR_BGR2GRAY);
		tempGray.convertTo(gray2, CV_32FC1);
		img2.Allocate(w, h, cudaSift::iAlignUp(w, 128), false, nullptr, (float*) gray2.data);
		img2.stream = 0;
		img2.Download();
		cudaSift::ExtractSift(siftData2, img2, 5, initBlur, thresh, 0.0f, false);


		// TODO: Apply mask before performing matching
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
				if (sqrt(pow(sp.xpos - spm.xpos, 2) + pow(sp.ypos - spm.ypos, 2)) > pixel_motion_thresh) { continue; }

				cv::Point2i pt1(sp.xpos, sp.ypos);
				cv::Point2i pt2(spm.xpos, spm.ypos);

				// Check mask
				if (!maskPrev.at<uint8_t>(pt1) || !maskCurr.at<uint8_t>(pt2))
				{
					continue;
				}

				// Get depths and validity
				uint16_t dVal1 = buffer.depth.at(tPrev)->image.at<uint16_t>(pt1);
				uint16_t dVal2 = buffer.depth.at(tCurr)->image.at<uint16_t>(pt2);
				if (!(DepthTraits::valid(dVal1) && DepthTraits::valid(dVal2))) { continue; }

				// Get 3D motion vector
				float depth1 = DepthTraits::toMeters(dVal1); // if (depth1 > maxZ || depth1 < minZ) { continue; }
				float depth2 = DepthTraits::toMeters(dVal2); // if (depth2 > maxZ || depth2 < minZ) { continue; }
				Vector p1 = toPoint3D<Vector>(sp.xpos, sp.ypos, depth1, buffer.cameraModel);
				Vector p2 = toPoint3D<Vector>(spm.xpos, spm.ypos, depth2, buffer.cameraModel);
				Vector v = p2 - p1;
				const double dist_thresh = track_options.meterRadius;
				if ((v.x() * v.x() + v.y() * v.y() + v.z() * v.z()) > dist_thresh * dist_thresh) { continue; }

				// Draw the match
				cv::circle(display, cv::Point2f(sp.xpos, sp.ypos), sp.scale, cv::Scalar(255, 0, 0));
				cv::circle(display, cv::Point2f(spm.xpos, spm.ypos), spm.scale, cv::Scalar(0, 255, 0));
				cv::line(display, cv::Point2f(sp.xpos, sp.ypos), cv::Point2f(spm.xpos, spm.ypos),
				         cv::Scalar(0, 0, 255));

				flow2.push_back({pt1, pt2});
				flow3.push_back({p1, v});
			}
		}

		visualization_msgs::MarkerArray ma;
		ma.markers.push_back(visualizeFlow(flow3));
		vizPub.publish(ma);

		flows2[{tPrev, tCurr}] = flow2;
		flows3[{tPrev, tCurr}] = flow3;

		if (shouldRecord)
		{
//			video->write(buffer.rgb.at(tPrev)->image);
//			tracking->write(display);
//			cv::namedWindow("Matches", cv::WINDOW_NORMAL);
//			cv::namedWindow("Mask", cv::WINDOW_NORMAL);
//
//			cv::Mat display1(display.size(), CV_8UC3, cv::Scalar(0, 0, 0));
//			display1.setTo(cv::Scalar(255, 0, 0), maskPrev);
//			cv::Mat display2(display.size(), CV_8UC3, cv::Scalar(0, 0, 0));
//			display2.setTo(cv::Scalar(0, 0, 255), maskCurr);
//
//			cv::imshow("Matches", display);
//			cv::imshow("Mask", display1 + display2);
//			cv::waitKey(0);
		}

		std::swap(gray1, gray2);
		std::swap(img1, img2);
		std::swap(siftData1, siftData2);
		cv::swap(maskPrev, maskCurr);

		img2.Destroy();
	}

	cudaSift::FreeSiftData(siftData1);
	cudaSift::FreeSiftData(siftData2);

	if (shouldRecord)
	{
		video->write(buffer.rgb.at(tLast)->image);
		video->release();
		tracking->release();
	}
}

}

#endif //#ifdef HAS_CUDA_SIFT
