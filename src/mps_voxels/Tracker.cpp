//
// Created by arprice on 8/15/18.
//

#include "mps_voxels/Tracker.h"

#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <tf/transform_listener.h>

#include <numeric>

void drawKeypoint(cv::Mat& display, const cv::KeyPoint& kp, const cv::Scalar& color)
{
	int radius = cvRound(kp.size/2); // KeyPoint::size is a diameter
	cv::circle(display, kp.pt, radius, color);
	if( kp.angle != -1 )
	{
		float srcAngleRad = kp.angle*(float)CV_PI/180.f;
		cv::Point2f orient( cvRound(cos(srcAngleRad)*radius ),
		                    cvRound(sin(srcAngleRad)*radius )
		);
		line(display, kp.pt, kp.pt+orient, color, 1, cv::LINE_AA);
	}
}

Tracker::Tracker(tf::TransformListener* _listener, const size_t _buffer, SubscriptionOptions _options, TrackingOptions _track_options)
	: MAX_BUFFER_LEN(_buffer), options(std::move(_options)), track_options(std::move(_track_options)),
	  listener(_listener),
	  callback_queue(),
	  spinner(1, &callback_queue)
{
	options.nh.setCallbackQueue(&callback_queue);
	options.pnh.setCallbackQueue(&callback_queue);

//	rgb_buffer.reserve(MAX_BUFFER_LEN);
//	depth_buffer.reserve(MAX_BUFFER_LEN);

//	cv::namedWindow("Tracking", cv::WINDOW_GUI_NORMAL);

	vizPub = options.nh.advertise<visualization_msgs::MarkerArray>("flow", 10, true);

	it = std::make_unique<image_transport::ImageTransport>(options.nh);
	rgb_sub = std::make_unique<image_transport::SubscriberFilter>(*it, options.rgb_topic, options.buffer, options.hints);
	depth_sub = std::make_unique<image_transport::SubscriberFilter>(*it, options.depth_topic, options.buffer, options.hints);
	cam_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(options.nh, options.cam_topic, options.buffer);

	sync = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(options.buffer), *rgb_sub, *depth_sub, *cam_sub);
	sync->registerCallback(boost::bind(&Tracker::imageCb, this, _1, _2, _3));

	spinner.start();
}

Tracker::~Tracker()
{
	stopCapture();
	reset();

	options.nh.shutdown();
	options.pnh.shutdown();

	spinner.stop();
	callback_queue.disable();
	callback_queue.clear();
}

void Tracker::startCapture()
{
	reset();

	auto joint_sub_options = ros::SubscribeOptions::create<sensor_msgs::JointState>(options.joint_topic, 2, boost::bind(&Tracker::jointCb, this, _1), ros::VoidPtr(), &callback_queue);
	joint_sub = std::make_unique<ros::Subscriber>(options.nh.subscribe(joint_sub_options));

	rgb_sub->subscribe(*it, options.rgb_topic, options.buffer, options.hints);
	depth_sub->subscribe(*it, options.depth_topic, options.buffer, options.hints);
	cam_sub->subscribe(options.nh, options.cam_topic, options.buffer);
}

void Tracker::stopCapture()
{
	rgb_sub->unsubscribe();
	depth_sub->unsubscribe();
	cam_sub->unsubscribe();
	if (joint_sub)
	{
		joint_sub->shutdown();
		joint_sub.reset();
	}
}

void Tracker::reset()
{
	rgb_buffer.clear();
	depth_buffer.clear();
	joint_buffer.clear();
	flows2.clear();
	flows3.clear();
}

cv::Mat& Tracker::getMask()
{
	////////////////////////////////////////
	//// Set up Mask
	////////////////////////////////////////

	if (!listener->waitForTransform(cameraModel.tfFrame(), "table_surface", ros::Time(0), ros::Duration(1.0)))
	{
		ROS_WARN_STREAM("Tracking failed: Failed to look up transform between '" << cameraModel.tfFrame() << "' and '" << "table_surface" << "'.");
		mask = cv::Mat::ones(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_8UC1);
	}
	else
	{
		tf::StampedTransform cameraTworld;
		listener->lookupTransform(cameraModel.tfFrame(), "table_surface", ros::Time(0), cameraTworld);

		mask = this->track_options.roi.getMask(cameraTworld, cameraModel);
	}

	return mask;
}

void Tracker::track(const std::vector<ros::Time>& steps)
{
	if (rgb_buffer.empty() || depth_buffer.empty())
	{
		ROS_WARN_STREAM("Tracking failed: Capture buffer empty.");
		return;
	}

	mask = getMask();

	cv::imshow("Tracking", mask);
	cv::waitKey(100);

	////////////////////////////////////////
	//// Set up Keypoint tracking
	////////////////////////////////////////

	cv::Mat display;
	std::vector<cv::KeyPoint> kpts1, kpts2;
	cv::UMat gray1, gray2, desc1, desc2;

	const ros::Time& tFirst = steps.front();
	const ros::Time& tLast = steps.back();

	double fps = steps.size()/(tLast - tFirst).toSec();
	cv::VideoWriter video("source.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer[tFirst]->image.size(), true);
	cv::VideoWriter tracking("tracking.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer[tFirst]->image.size(), true);

//	auto detector = cv::AKAZE::create();
//	auto detector = cv::KAZE::create();
	auto detector = cv::xfeatures2d::SIFT::create();

	cv::cvtColor(rgb_buffer[tFirst]->image, gray1, cv::COLOR_BGR2GRAY);
	detector->detectAndCompute(gray1, mask, kpts1, desc1);
	for (int i = 1; i < static_cast<int>(steps.size()) && ros::ok(); ++i)
	{
		const ros::Time& tPrev = steps[i-1];
		const ros::Time& tCurr = steps[i];

		rgb_buffer[tPrev]->image.copyTo(display);
		cv::cvtColor(rgb_buffer[tCurr]->image, gray2, cv::COLOR_BGR2GRAY);

		double detectorStartTime = (double)cv::getTickCount();
		detector->detectAndCompute(gray2, mask, kpts2, desc2);
		double detectorEndTime = (double)cv::getTickCount();
		std::cerr << "Detect: " << (detectorEndTime - detectorStartTime)/cv::getTickFrequency() << std::endl;

		double matchStartTime = (double)cv::getTickCount();
		cv::BFMatcher matcher(cv::NORM_L2);//(cv::NORM_HAMMING);
		std::vector< std::vector<cv::DMatch> > nn_matches;
//	    matcher.knnMatch(desc1, desc2, nn_matches, 3);
		matcher.radiusMatch(desc1, desc2, nn_matches, track_options.featureRadius);
		double matchEndTime = (double)cv::getTickCount();
		std::cerr << "Match: " << (matchEndTime - matchStartTime)/cv::getTickFrequency() << std::endl;

		Flow2D flow2;
		Flow3D flow3;

		for (int ii = 0; ii < static_cast<int>(nn_matches.size()); ++ii)
		{
			nn_matches[ii].erase(std::remove_if(nn_matches[ii].begin(), nn_matches[ii].end(),
			                                    [&](const cv::DMatch& match){
				                                    cv::Point diff = kpts2[match.trainIdx].pt - kpts1[match.queryIdx].pt;
				                                    return std::sqrt(diff.x*diff.x + diff.y*diff.y) > track_options.pixelRadius;
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

				uint16_t dVal1 = depth_buffer[tPrev]->image.at<uint16_t>(kp1.pt);
				uint16_t dVal2 = depth_buffer[tCurr]->image.at<uint16_t>(kp2.pt);
				if (!(DepthTraits::valid(dVal1) && DepthTraits::valid(dVal2))) { continue; }

				float depth1 = DepthTraits::toMeters(dVal1); // if (depth1 > maxZ || depth1 < minZ) { continue; }
				float depth2 = DepthTraits::toMeters(dVal2); // if (depth2 > maxZ || depth2 < minZ) { continue; }
				Vector p1 = toPoint3D<Vector>(kp1.pt.x, kp1.pt.y, depth1, cameraModel);
				Vector p2 = toPoint3D<Vector>(kp2.pt.x, kp2.pt.y, depth2, cameraModel);
				Vector v = p2-p1;
				const double dist_thresh = track_options.meterRadius;
				if ((v.x()*v.x() + v.y()*v.y() + v.z()*v.z()) > dist_thresh*dist_thresh) { continue; }
				flow3.push_back({p1, v});
				flow2.push_back({kp1.pt, kp2.pt});
			}
		}

		visualization_msgs::MarkerArray ma;
		ma.markers.push_back(visualizeFlow(flow3));
		vizPub.publish(ma);

		flows2[{tPrev, tCurr}] = flow2;
		flows3[{tPrev, tCurr}] = flow3;

		video.write(rgb_buffer[tPrev]->image);
		tracking.write(display);
		cv::imshow("Tracking", display);
		cv::waitKey(1);

		std::swap(gray1, gray2);
		std::swap(kpts1, kpts2);
		std::swap(desc1, desc2);
	}

	video.write(rgb_buffer[tLast]->image);
	tracking.release();
}

void Tracker::siamtrack(LabelT label, const std::vector<ros::Time>& steps, mps_msgs::AABBox2d bbox)
{
	Tracker::track(steps);
}

void Tracker::imageCb(const sensor_msgs::ImageConstPtr& rgb_msg,
                      const sensor_msgs::ImageConstPtr& depth_msg,
                      const sensor_msgs::CameraInfoConstPtr& cam_msg)
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

	cameraModel.fromCameraInfo(*cam_msg);

	if (rgb_buffer.size() < MAX_BUFFER_LEN)
	{
		rgb_buffer.insert({cv_rgb_ptr->header.stamp, cv_rgb_ptr});
		std::cerr << rgb_buffer.size() << ": " << rgb_buffer.rbegin()->first - rgb_buffer.begin()->first << std::endl;
	}

	if (depth_buffer.size() < MAX_BUFFER_LEN)
	{
		depth_buffer.insert({cv_depth_ptr->header.stamp, cv_depth_ptr});
	}

	if (rgb_buffer.size() == MAX_BUFFER_LEN)
	{
		stopCapture();
	}
}

void Tracker::jointCb(const sensor_msgs::JointStateConstPtr& joint_msg)
{
	joint_buffer.insert(joint_buffer.end(), {joint_msg->header.stamp, joint_msg});
}

bool estimateRigidTransform(const Tracker::Flow3D& flow, Eigen::Isometry3d& bTa)
{
	const int RANSAC_ITERS = 100;
	const double MATCH_DISTANCE = 0.01;
	const int VOTE_THRESH = RANSAC_ITERS/10;

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
			B.col(s) = flow[indices[s]].first+flow[indices[s]].second;
		}
		Eigen::Matrix4d putativeM = Eigen::umeyama(A, B, false);
		Eigen::Affine3d putativeT;
		putativeT.linear() = putativeM.topLeftCorner<3, 3>();
		putativeT.translation() = putativeM.topRightCorner<3, 1>();

		for (size_t i = 0; i < N; ++i)
		{
			if (((putativeT*flow[i].first)-(flow[i].first-flow[i].second)).norm() <= MATCH_DISTANCE)
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
		B.col(s) = flow[inliers[s]].first+flow[inliers[s]].second;
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