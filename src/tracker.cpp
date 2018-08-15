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

#include <tf/transform_listener.h>
#include <visualization_msgs/MarkerArray.h>
#include "mps_voxels/colormap.h"

#include <ros/ros.h>

std::shared_ptr<tf::TransformListener> listener;
ros::Publisher vizPub;

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

template <typename PointT>
visualization_msgs::Marker visualizeAABB(const PointT& minExtent, const PointT& maxExtent)
{
	visualization_msgs::Marker m;
	m.type = visualization_msgs::Marker::CUBE;
	m.action = visualization_msgs::Marker::ADD;
	m.frame_locked = true;
	m.header.frame_id = "table_surface";
	m.header.stamp = ros::Time::now();
	m.pose.orientation.w = 1.0f;
	m.pose.position.x = (minExtent.x() + maxExtent.x())/2.0f;
	m.pose.position.y = (minExtent.y() + maxExtent.y())/2.0f;
	m.pose.position.z = (minExtent.z() + maxExtent.z())/2.0f;
	m.scale.x = (maxExtent.x() - minExtent.x());
	m.scale.y = (maxExtent.y() - minExtent.y());
	m.scale.z = (maxExtent.z() - minExtent.z());
	m.color.a = 0.5;
	m.color.b = 1.0;
	m.id = 0;
	m.ns = "aabb";
	return m;
}

template <typename PointT>
visualization_msgs::Marker visualizeFlow(const std::vector<std::pair<PointT, PointT>>& flow)
{
	visualization_msgs::Marker m;
	m.type = visualization_msgs::Marker::LINE_LIST;
	m.action = visualization_msgs::Marker::ADD;
	m.frame_locked = true;
	m.header.frame_id = "kinect2_victor_head_rgb_optical_frame";
	m.header.stamp = ros::Time::now();
	m.pose.orientation.w = 1.0f;
	m.scale.x = 0.002;
//	m.scale.y = 1;
//	m.scale.z = 1;
	m.color.a = 1.0;
	m.color.r = 1.0;
	m.id = 0;
	m.ns = "flow";
	for (const auto& f : flow)
	{
		geometry_msgs::Point pt;
		pt.x = f.first.x;
		pt.y = f.first.y;
		pt.z = f.first.z;

		m.points.push_back(pt);

		pt.x += f.second.x;
		pt.y += f.second.y;
		pt.z += f.second.z;

		m.points.push_back(pt);

		float mag = std::sqrt(f.second.x*f.second.x + f.second.y*f.second.y + f.second.z*f.second.z)/0.0125;
		std_msgs::ColorRGBA color;
		color.a = 1.0;
		colormap(igl::inferno_cm, mag, color.r, color.g, color.b);

		m.colors.push_back(color);
		m.colors.push_back(color);
	}
	return m;
}

template <typename PointT>
PointT toPoint3D(const float uIdx, const float vIdx, const float depthMeters, const image_geometry::PinholeCameraModel& cam)
{
	PointT pt;
	pt.x = (uIdx - cam.cx()) * depthMeters / cam.fx();
	pt.y = (vIdx - cam.cy()) * depthMeters / cam.fy();
	pt.z = depthMeters;
	return pt;
}

template <typename PointT>
std::vector<PointT> getCorners(const PointT& min, const PointT& max, const int dimension)
{
	// There will be 2^DIM corners to deal with
	const unsigned nCorners = (1u << dimension);
	std::vector<PointT> corners(nCorners);
	for (unsigned perm = 0; perm < nCorners; ++perm)
	{
		PointT pt;
		for (int d = 0; d < dimension; ++d)
		{
			pt[d] = (perm & (1u<<d)) ? min[d] : max[d];
		}
		corners[perm] = pt;
	}
	return corners;
}

class Tracker
{
public:
	using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
	using DepthTraits = depth_image_proc::DepthTraits<uint16_t>;

	struct SubscriptionOptions
	{
		ros::NodeHandle nh, pnh;
		image_transport::ImageTransport it;
		image_transport::TransportHints hints;
		int buffer;
		std::string topic_prefix;
		std::string rgb_topic;
		std::string depth_topic;
		std::string cam_topic;
		SubscriptionOptions()
			: nh(), pnh("~"),
			  it(nh), hints("compressed", ros::TransportHints(), pnh),
			  buffer(10), topic_prefix("/kinect2_victor_head/hd"),
			  rgb_topic(topic_prefix+"/image_color_rect"),
			  depth_topic(topic_prefix+"/image_depth_rect"),
			  cam_topic(topic_prefix+"/camera_info")
		{
		}
	};

	struct TrackingOptions
	{
		float featureRadius;
		float pixelRadius;
		float meterRadius;

		TrackingOptions() : featureRadius(250.0f), pixelRadius(30.0f), meterRadius(0.05)
		{
		}
	};

	const size_t MAX_BUFFER_LEN;//1000;

	std::vector<cv_bridge::CvImagePtr> rgb_buffer;
	std::vector<cv_bridge::CvImagePtr> depth_buffer;
	sensor_msgs::CameraInfo cam;
	image_geometry::PinholeCameraModel cameraModel;

	std::unique_ptr<image_transport::SubscriberFilter> rgb_sub;
	std::unique_ptr<image_transport::SubscriberFilter> depth_sub;
	std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> cam_sub;
	std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

	cv::Mat mask;
	double minZ;
	double maxZ;

	SubscriptionOptions options;
	TrackingOptions track_options;

	explicit
	Tracker(const size_t _buffer = 500, SubscriptionOptions _options = SubscriptionOptions())
		: MAX_BUFFER_LEN(_buffer), options(std::move(_options))
	{
		rgb_buffer.reserve(MAX_BUFFER_LEN);
		depth_buffer.reserve(MAX_BUFFER_LEN);

		cv::namedWindow("prev", cv::WINDOW_GUI_NORMAL);

		rgb_sub = std::make_unique<image_transport::SubscriberFilter>(options.it, options.rgb_topic, options.buffer, options.hints);
		depth_sub = std::make_unique<image_transport::SubscriberFilter>(options.it, options.depth_topic, options.buffer, options.hints);
		cam_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(options.nh, options.cam_topic, options.buffer);

		sync = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(options.buffer), *rgb_sub, *depth_sub, *cam_sub);
		sync->registerCallback(boost::bind(&Tracker::imageCb, this, _1, _2, _3));
	}

	void startCapture()
	{
		rgb_buffer.clear();
		depth_buffer.clear();
		rgb_sub->subscribe(options.it, options.rgb_topic, options.buffer, options.hints);
		depth_sub->subscribe(options.it, options.depth_topic, options.buffer, options.hints);
		cam_sub->subscribe(options.nh, options.cam_topic, options.buffer);
	}

	void stopCapture()
	{
		rgb_sub->unsubscribe();
		depth_sub->unsubscribe();
		cam_sub->unsubscribe();
	}

	void track()
	{
		cv::Mat display;
		std::vector<cv::KeyPoint> kpts1, kpts2;
		cv::UMat gray1, gray2, desc1, desc2;

		double fps = MAX_BUFFER_LEN/(rgb_buffer.back()->header.stamp - rgb_buffer.front()->header.stamp).toSec();
		cv::VideoWriter video("source.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer.front()->image.size(), true);
		cv::VideoWriter tracking("tracking.avi", CV_FOURCC('M', 'J', 'P', 'G'), fps, rgb_buffer.front()->image.size(), true);

//	    auto detector = cv::AKAZE::create();
//	    auto detector = cv::KAZE::create();
		auto detector = cv::xfeatures2d::SIFT::create();

		cv::cvtColor(rgb_buffer.front()->image, gray1, cv::COLOR_BGR2GRAY);
		detector->detectAndCompute(gray1, mask, kpts1, desc1);
		for (int i = 1; i < static_cast<int>(rgb_buffer.size()) && ros::ok(); ++i)
		{
			rgb_buffer[i-1]->image.copyTo(display);
			cv::cvtColor(rgb_buffer[i]->image, gray2, cv::COLOR_BGR2GRAY);

			double detectorStartTime = (double)cv::getTickCount();
			detector->detectAndCompute(gray2, mask, kpts2, desc2);
			double detectorEndTime = (double)cv::getTickCount();
			std::cerr << "Detect: " << (detectorEndTime - detectorStartTime)/cv::getTickFrequency() << std::endl;

			double matchStartTime = (double)cv::getTickCount();
			cv::BFMatcher matcher(cv::NORM_L2);//(cv::NORM_HAMMING);
			std::vector< std::vector<cv::DMatch> > nn_matches;
//	    	matcher.knnMatch(desc1, desc2, nn_matches, 3);
			matcher.radiusMatch(desc1, desc2, nn_matches, track_options.featureRadius);
			double matchEndTime = (double)cv::getTickCount();
			std::cerr << "Match: " << (matchEndTime - matchStartTime)/cv::getTickFrequency() << std::endl;

			using Vector = cv::Point3d;
			std::vector<std::pair<Vector, Vector>> flow;

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

					uint16_t dVal1 = depth_buffer[i-1]->image.at<uint16_t>(kp1.pt);
					uint16_t dVal2 = depth_buffer[i]->image.at<uint16_t>(kp2.pt);
					if (!(DepthTraits::valid(dVal1) && DepthTraits::valid(dVal2))) { continue; }

					float depth1 = DepthTraits::toMeters(dVal1); if (depth1 > maxZ || depth1 < minZ) { continue; }
					float depth2 = DepthTraits::toMeters(dVal2); if (depth2 > maxZ || depth2 < minZ) { continue; }
					Vector p1 = toPoint3D<Vector>(kp1.pt.x, kp1.pt.y, depth1, cameraModel);
					Vector p2 = toPoint3D<Vector>(kp2.pt.x, kp2.pt.y, depth2, cameraModel);
					Vector v = p2-p1;
					const double dist_thresh = track_options.meterRadius;
					if ((v.x*v.x + v.y*v.y + v.z*v.z) > dist_thresh*dist_thresh) { continue; }
					flow.push_back({p1, v});
				}
			}

			visualization_msgs::MarkerArray ma;
			ma.markers.push_back(visualizeFlow(flow));
			vizPub.publish(ma);

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
			rgb_buffer.push_back(cv_rgb_ptr);
			std::cerr << rgb_buffer.size() << ": " << rgb_buffer.back()->header.stamp - rgb_buffer.front()->header.stamp << std::endl;
		}

		if (depth_buffer.size() < MAX_BUFFER_LEN)
		{
			depth_buffer.push_back(cv_depth_ptr);
		}

		if (rgb_buffer.size() == MAX_BUFFER_LEN)
		{
			stopCapture();
			const auto& img = rgb_buffer.back()->image;
			mask = cv::Mat::zeros(img.size(), CV_8UC1);


			if (!listener->waitForTransform(rgb_buffer.back()->header.frame_id, "table_surface", ros::Time(0), ros::Duration(5.0)))
			{
				ROS_WARN_STREAM("Failed to look up transform between '" << rgb_buffer.back()->header.frame_id << "' and '" << "table_surface" << "'.");
				return;
			}
			tf::StampedTransform cameraTworld;
			listener->lookupTransform(cameraModel.tfFrame(), "table_surface", ros::Time(0), cameraTworld);

			tf::Vector3 minExtent(-0.4f, -0.6f, -0.05f);
			tf::Vector3 maxExtent(0.4f, 0.6f, 0.4f);

			visualization_msgs::MarkerArray ma;
			ma.markers.push_back(visualizeAABB(minExtent, maxExtent));
			vizPub.publish(ma);

			std::vector<cv::Point2i> boxCorners;
			minZ = std::numeric_limits<double>::max();
			maxZ = std::numeric_limits<double>::lowest();
			for (const tf::Vector3& pt_world : getCorners(minExtent, maxExtent, 3))
			{
				tf::Vector3 pt_camera = cameraTworld * pt_world;
				minZ = std::min(minZ, pt_camera.z());
				maxZ = std::max(maxZ, pt_camera.z());
				cv::Point2d imgPt = cameraModel.project3dToPixel({pt_camera.x(), pt_camera.y(), pt_camera.z()});
				boxCorners.push_back(imgPt);
			}
			std::vector<cv::Point2i> hullPoly;
			cv::convexHull(boxCorners, hullPoly);

			cv::fillConvexPoly(mask, hullPoly, cv::Scalar::all(255));

			cv::Mat depthMask = depth_buffer.back()->image > DepthTraits::fromMeters(minZ)
			                    & depth_buffer.back()->image < DepthTraits::fromMeters(maxZ);
			cv::dilate(depthMask, depthMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25,25)));
			cv::erode(depthMask, depthMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25,25)));

			mask &= depthMask;
			cv::imshow("prev", depthMask);
			cv::waitKey(100);
			cv::imshow("prev", mask);
			cv::waitKey(100);
			track();
		}

		cam = *cam_msg;
	}
};

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
	ros::NodeHandle nh;

	vizPub = nh.advertise<visualization_msgs::MarkerArray>("roi", 10, true);
	listener = std::make_shared<tf::TransformListener>();

	Tracker t;

	ros::spin();

	return 0;
}