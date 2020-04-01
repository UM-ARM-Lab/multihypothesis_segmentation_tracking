//
// Created by arprice on 8/15/18.
//

#include "mps_voxels/Tracker.h"
#include "mps_voxels/project_point.hpp"

#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <numeric>

namespace mps
{

void drawKeypoint(cv::Mat& display, const cv::KeyPoint& kp, const cv::Scalar& color)
{
	int radius = cvRound(kp.size / 2); // KeyPoint::size is a diameter
	cv::circle(display, kp.pt, radius, color);
	if (kp.angle != -1)
	{
		float srcAngleRad = kp.angle * (float) CV_PI / 180.f;
		cv::Point2f orient(cvRound(cos(srcAngleRad) * radius),
		                   cvRound(sin(srcAngleRad) * radius)
		);
		line(display, kp.pt, kp.pt + orient, color, 1, cv::LINE_AA);
	}
}

Tracker::Tracker(TrackingOptions _options)
	:track_options(std::move(_options))
{
	vizPub = ros::NodeHandle().advertise<visualization_msgs::MarkerArray>("flow", 10, true);
}

void Tracker::reset()
{
	flows2.clear();
	flows3.clear();
}

cv::Mat& Tracker::getMask(const SensorHistoryBuffer& buffer)
{
	////////////////////////////////////////
	//// Set up Mask
	////////////////////////////////////////

	const std::string tableFrame = "table_surface";

	if (!buffer.tfs
	    || !buffer.tfs->canTransform(buffer.cameraModel.tfFrame(), tableFrame, ros::Time(0)))
	{
		ROS_WARN_STREAM("Tracking failed: Failed to look up transform between '" << buffer.cameraModel.tfFrame() << "' and '"
		                                                                         << tableFrame << "'.");
		mask = cv::Mat::ones(buffer.cameraModel.cameraInfo().height, buffer.cameraModel.cameraInfo().width, CV_8UC1);
	}
	else
	{
		tf::StampedTransform cameraTworld;
		geometry_msgs::TransformStamped cTw = buffer.tfs->lookupTransform(buffer.cameraModel.tfFrame(), tableFrame, ros::Time(0));
		tf::transformStampedMsgToTF(cTw, cameraTworld);

		mask = this->track_options.roi.getMask(cameraTworld, buffer.cameraModel);
	}

	return mask;
}

void Tracker::track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const std::map<ros::Time, cv::Mat>& masks, const std::string directory)
{
	if (steps.size() > masks.size())
	{
		ROS_ERROR_STREAM("Sparse tracking failed: # of timesteps: " << steps.size() << " is more than # of masks: " << masks.size() << ". Return!");
		return;
	}
	for (auto& t : steps)
	{
		if (masks.find(t) == masks.end()) { ROS_ERROR_STREAM("Sparse tracking failed: Mask does not contain all requested timesteps. Return!");
											return; }
	}
	if (buffer.rgb.empty() || buffer.depth.empty())
	{
		ROS_WARN_STREAM("Tracking failed: Capture buffer empty.");
		return;
	}

	mask = getMask(buffer);

//	cv::imshow("Tracking", mask);
//	cv::waitKey(100);

	////////////////////////////////////////
	//// Set up Keypoint tracking
	////////////////////////////////////////

	cv::Mat display;
	std::vector<cv::KeyPoint> kpts1, kpts2;
	cv::UMat gray1, gray2, desc1, desc2;

	const ros::Time& tFirst = steps.front();
	const ros::Time& tLast = steps.back();

	double fps = 1.0;// steps.size() / (tLast - tFirst).toSec();
	std::string sourceVideo, trackingVideo;
	if (directory == " ") { sourceVideo = "source.avi"; trackingVideo = "tracking.avi"; }
	else { sourceVideo = directory + "source.avi"; trackingVideo = directory + "tracking.avi"; }
	cv::VideoWriter video(sourceVideo, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, buffer.rgb.at(tFirst)->image.size(), true);
	cv::VideoWriter tracking(trackingVideo, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps, buffer.rgb.at(tFirst)->image.size(),
	                         true);

//	auto detector = cv::AKAZE::create();
//	auto detector = cv::KAZE::create();
	auto detector = cv::xfeatures2d::SIFT::create();

	cv::Mat instanceMask;
	if (!masks.empty()) { instanceMask = mask & masks.at(tFirst); }
	else { instanceMask = mask; }

	// TODO: add a visualization flag or something
	visualization_msgs::MarkerArray ma;

	cv::cvtColor(buffer.rgb.at(tFirst)->image, gray1, cv::COLOR_BGR2GRAY);
	detector->detectAndCompute(gray1, instanceMask, kpts1, desc1);
	for (int i = 1; i < static_cast<int>(steps.size()) && ros::ok(); ++i)
	{
		const ros::Time& tPrev = steps[i - 1];
		const ros::Time& tCurr = steps.at(i);

		buffer.rgb.at(tPrev)->image.copyTo(display);
		cv::cvtColor(buffer.rgb.at(tCurr)->image, gray2, cv::COLOR_BGR2GRAY);

		if (!masks.empty()) { instanceMask = mask & masks.at(tCurr); }
		else { instanceMask = mask; }

		double detectorStartTime = (double) cv::getTickCount();
		detector->detectAndCompute(gray2, instanceMask, kpts2, desc2);
		double detectorEndTime = (double) cv::getTickCount();
		std::cerr << "Detect: " << (detectorEndTime - detectorStartTime) / cv::getTickFrequency() << std::endl;

		if (kpts1.empty() || kpts2.empty())
		{
			// No features detected on this pass, skip ahead
			std::swap(gray1, gray2);
			std::swap(kpts1, kpts2);
			std::swap(desc1, desc2);
			continue;
		}

//		double matchStartTime = (double) cv::getTickCount();
		cv::BFMatcher matcher(cv::NORM_L2);//(cv::NORM_HAMMING);
		std::vector<std::vector<cv::DMatch>> nn_matches;
//	    matcher.knnMatch(desc1, desc2, nn_matches, 3);
		matcher.radiusMatch(desc1, desc2, nn_matches, track_options.featureRadius);
//		double matchEndTime = (double) cv::getTickCount();
//		std::cerr << "Match: " << (matchEndTime - matchStartTime) / cv::getTickFrequency() << std::endl;

		Flow2D flow2;
		Flow3D flow3;

		for (int ii = 0; ii < static_cast<int>(nn_matches.size()); ++ii)
		{
			nn_matches[ii].erase(std::remove_if(nn_matches[ii].begin(), nn_matches[ii].end(),
			                                    [&](const cv::DMatch& match)
			                                    {
				                                    cv::Point diff = kpts2[match.trainIdx].pt
				                                                     - kpts1[match.queryIdx].pt;
				                                    return std::sqrt(diff.x * diff.x + diff.y * diff.y)
				                                           > track_options.pixelRadius;
			                                    }),
			                     nn_matches[ii].end());

			for (int jj = 0; jj < static_cast<int>(nn_matches[ii].size()); ++jj)
			{
				const cv::DMatch& match = nn_matches[ii][jj];
				const auto& kp1 = kpts1[match.queryIdx];
				const auto& kp2 = kpts2[match.trainIdx];
				drawKeypoint(display, kp1, cv::Scalar(255, 0, 0));
				drawKeypoint(display, kp2, cv::Scalar(0, 255, 0));
				cv::arrowedLine(display, kp1.pt, kp2.pt, cv::Scalar(0, 0, 255));

				uint16_t dVal1 = buffer.depth.at(tPrev)->image.at<uint16_t>(kp1.pt);
				uint16_t dVal2 = buffer.depth.at(tCurr)->image.at<uint16_t>(kp2.pt);
				if (!(DepthTraits::valid(dVal1) && DepthTraits::valid(dVal2))) { continue; }

				float depth1 = DepthTraits::toMeters(dVal1); // if (depth1 > maxZ || depth1 < minZ) { continue; }
				float depth2 = DepthTraits::toMeters(dVal2); // if (depth2 > maxZ || depth2 < minZ) { continue; }
				Vector p1 = toPoint3D<Vector>(kp1.pt.x, kp1.pt.y, depth1, buffer.cameraModel);
				Vector p2 = toPoint3D<Vector>(kp2.pt.x, kp2.pt.y, depth2, buffer.cameraModel);
				Vector v = p2 - p1;
				const double dist_thresh = track_options.meterRadius;
				if ((v.x() * v.x() + v.y() * v.y() + v.z() * v.z()) > dist_thresh * dist_thresh) { continue; }
				flow3.push_back({p1, v});
				flow2.push_back({kp1.pt, kp2.pt});
			}
		}

		ma.markers.push_back(visualizeFlow(flow3));
		vizPub.publish(ma);

		flows2[{tPrev, tCurr}] = flow2;
		flows3[{tPrev, tCurr}] = flow3;

		video.write(buffer.rgb.at(tPrev)->image);
		tracking.write(display);
//		cv::imshow("Tracking", display);
//		cv::waitKey(1);

		std::swap(gray1, gray2);
		std::swap(kpts1, kpts2);
		std::swap(desc1, desc2);
	}

	video.write(buffer.rgb.at(tLast)->image);
	tracking.release();
}

/*
void Tracker::siftOnMask(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, LabelT label)
{
	if (buffer.rgb.empty() || buffer.depth.empty())
	{
		ROS_WARN_STREAM("SIFT failed: Capture buffer empty.");
		return;
	}
	if (steps.size() != labelToMasksLookup[label].size() + 1)
	{
		ROS_WARN_STREAM("Timesteps do not match with Masks!!!");
		std::cerr << "Number of timestamps: " << steps.size() << std::endl;
		std::cerr << "Number of masks: " << labelToMasksLookup[label].size() << std::endl;
		return;
	}

	mask = getMask(buffer);

//	cv::imshow("Tracking", mask);
//	cv::waitKey(100);

	////////////////////////////////////////
	//// Set up Keypoint tracking
	////////////////////////////////////////

	cv::Mat display;
	std::vector<cv::KeyPoint> kpts1, kpts2;
	cv::UMat gray1, gray2, desc1, desc2;

	int startTimestepIndex = 1; // SiamMask does not return the first frame mask
	const ros::Time& tFirst = steps[startTimestepIndex];
	const ros::Time& tLast = steps.back();

//	double fps = steps.size() / (tLast - tFirst).toSec();

	cv::VideoWriter video("/home/kunhuang/Videos/source.avi",
						cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
							1, buffer.rgb.at(tFirst)->image.size(), true);
	cv::VideoWriter tracking("/home/kunhuang/Videos/tracking_" + std::to_string((int)label) + ".avi",
							cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 1, buffer.rgb.at(tFirst)->image.size(),
	                         true);

//	auto detector = cv::AKAZE::create();
//	auto detector = cv::KAZE::create();
	auto detector = cv::xfeatures2d::SIFT::create();


	cv::cvtColor(maskImage(buffer.rgb.at(tFirst)->image, labelToMasksLookup[label][0]), gray1, cv::COLOR_BGR2GRAY);
	detector->detectAndCompute(gray1, mask, kpts1, desc1); // TODO: change mask
	for (int i = startTimestepIndex+1; i < static_cast<int>(steps.size()) && ros::ok(); ++i)
	{
		const ros::Time& tPrev = steps[i - 1];
		const ros::Time& tCurr = steps.at(i);

		maskImage(buffer.rgb.at(tPrev)->image, labelToMasksLookup[label][i - 2]).copyTo(display);
		cv::cvtColor(maskImage(buffer.rgb.at(tCurr)->image, labelToMasksLookup[label][i - 1]), gray2, cv::COLOR_BGR2GRAY);

		double detectorStartTime = (double) cv::getTickCount();
		detector->detectAndCompute(gray2, mask, kpts2, desc2);
		double detectorEndTime = (double) cv::getTickCount();
//		std::cerr << "Detect: " << (detectorEndTime - detectorStartTime) / cv::getTickFrequency() << std::endl;

		double matchStartTime = (double) cv::getTickCount();
		cv::BFMatcher matcher(cv::NORM_L2);//(cv::NORM_HAMMING);
		std::vector<std::vector<cv::DMatch>> nn_matches;
//	    matcher.knnMatch(desc1, desc2, nn_matches, 3);
		matcher.radiusMatch(desc1, desc2, nn_matches, track_options.featureRadius);
		double matchEndTime = (double) cv::getTickCount();
//		std::cerr << "Match: " << (matchEndTime - matchStartTime) / cv::getTickFrequency() << std::endl;

		Flow2D flow2;
		Flow3D flow3;

		for (int ii = 0; ii < static_cast<int>(nn_matches.size()); ++ii)
		{
			nn_matches[ii].erase(std::remove_if(nn_matches[ii].begin(), nn_matches[ii].end(),
			                                    [&](const cv::DMatch& match)
			                                    {
				                                    cv::Point diff = kpts2[match.trainIdx].pt
				                                                     - kpts1[match.queryIdx].pt;
				                                    return std::sqrt(diff.x * diff.x + diff.y * diff.y)
				                                           > track_options.pixelRadius;
			                                    }),
			                     nn_matches[ii].end());

			for (int jj = 0; jj < static_cast<int>(nn_matches[ii].size()); ++jj)
			{
				const cv::DMatch& match = nn_matches[ii][jj];
				const auto& kp1 = kpts1[match.queryIdx];
				const auto& kp2 = kpts2[match.trainIdx];
				drawKeypoint(display, kp1, cv::Scalar(255, 0, 0));
				drawKeypoint(display, kp2, cv::Scalar(0, 255, 0));
				cv::arrowedLine(display, kp1.pt, kp2.pt, cv::Scalar(0, 0, 255));

				uint16_t dVal1 = buffer.depth.at(tPrev)->image.at<uint16_t>(kp1.pt);
				uint16_t dVal2 = buffer.depth.at(tCurr)->image.at<uint16_t>(kp2.pt);
				if (!(DepthTraits::valid(dVal1) && DepthTraits::valid(dVal2))) { continue; }

				float depth1 = DepthTraits::toMeters(dVal1); // if (depth1 > maxZ || depth1 < minZ) { continue; }
				float depth2 = DepthTraits::toMeters(dVal2); // if (depth2 > maxZ || depth2 < minZ) { continue; }
				Vector p1 = toPoint3D<Vector>(kp1.pt.x, kp1.pt.y, depth1, buffer.cameraModel);
				Vector p2 = toPoint3D<Vector>(kp2.pt.x, kp2.pt.y, depth2, buffer.cameraModel);
				Vector v = p2 - p1;
				const double dist_thresh = track_options.meterRadius;
				if ((v.x() * v.x() + v.y() * v.y() + v.z() * v.z()) > dist_thresh * dist_thresh) { continue; }
				flow3.push_back({p1, v});
				flow2.push_back({kp1.pt, kp2.pt});
			}
		}
//		std::cerr << "Flow computed!" << std::endl;

//		visualization_msgs::MarkerArray ma;
//		ma.markers.push_back(visualizeFlow(flow3));
//		vizPub.publish(ma);
//		std::cerr << "Flow published!" << std::endl;

		flows2[{tPrev, tCurr}] = flow2;
		flows3[{tPrev, tCurr}] = flow3;

		video.write(buffer.rgb.at(tPrev)->image);
		tracking.write(display);
//		cv::imshow("Tracking", display);
//		cv::waitKey(1);

		std::swap(gray1, gray2);
		std::swap(kpts1, kpts2);
		std::swap(desc1, desc2);

//		std::cerr << "1 loop finished!" << std::endl;
	}
	std::cerr << "Video ready to be released!" << std::endl;

	video.write(buffer.rgb.at(tLast)->image);
	tracking.release();
}
*/

bool estimateRigidTransform(const Tracker::Flow3D& flow, Eigen::Isometry3d& bTa)
{
	const int RANSAC_ITERS = 100;
	const double MATCH_DISTANCE = 0.01;
	const int VOTE_THRESH = RANSAC_ITERS / 10;

	const size_t N = flow.size();
	std::vector<unsigned int> votes(N, 0);
	std::vector<unsigned int> inliers;
	inliers.reserve(N);

	// RANSAC
	#pragma omp parallel for
	for (int iter = 0; iter < RANSAC_ITERS; ++iter)
	{
		std::vector<unsigned int> indices(N);
		std::iota(indices.begin(), indices.end(), 0);
		std::random_shuffle(indices.begin(), indices.end());

		Eigen::Matrix3d A, B;
		for (int s = 0; s < 3; ++s)
		{
			A.col(s) = flow[indices[s]].first;
			B.col(s) = flow[indices[s]].first + flow[indices[s]].second;
		}
		Eigen::Matrix4d putativeM = Eigen::umeyama(A, B, false);
		Eigen::Affine3d putativeT;
		putativeT.linear() = putativeM.topLeftCorner<3, 3>();
		putativeT.translation() = putativeM.topRightCorner<3, 1>();

		for (size_t i = 0; i < N; ++i)
		{
			if (((putativeT * flow[i].first) - (flow[i].first - flow[i].second)).norm() <= MATCH_DISTANCE)
			{
				#pragma omp critical
				{
					++votes[i];
				}
			}
		}
	}

	for (unsigned int i = 0; i < N; ++i)
	{
		if (votes[i] >= VOTE_THRESH)
		{
			inliers.push_back(i);
		}
	}

	if (inliers.size() < 4)
	{
		return false;
	}

	Eigen::Matrix3Xd A(3, inliers.size()), B(3, inliers.size());
	for (size_t s = 0; s < inliers.size(); ++s)
	{
		A.col(s) = flow[inliers[s]].first;
		B.col(s) = flow[inliers[s]].first + flow[inliers[s]].second;
	}

//	Eigen::Matrix3Xd A(3, flow.size()), B(3, flow.size());
//	for (size_t s = 0; s < flow.size(); ++s)
//	{
//		A.col(s) = flow[s].first;
//		B.col(s) = flow[s].second;
//	}


	Eigen::Matrix4d putativeM = Eigen::umeyama(A, B, false);
	bTa = Eigen::Isometry3d::Identity();
	bTa.rotate(putativeM.topLeftCorner<3, 3>());
	bTa.translation() = putativeM.topRightCorner<3, 1>();

	return true;
}

}