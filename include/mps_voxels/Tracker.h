//
// Created by arprice on 8/15/18.
//

#ifndef MPS_VOXELS_TRACKER_H
#define MPS_VOXELS_TRACKER_H

#include "mps_voxels/ROI.h"

#include <opencv2/imgproc.hpp>

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

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include "mps_voxels/colormap.h"

#include <Eigen/Geometry>

void drawKeypoint(cv::Mat& display, const cv::KeyPoint& kp, const cv::Scalar& color);

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
		pt.x = f.first.x();
		pt.y = f.first.y();
		pt.z = f.first.z();

		m.points.push_back(pt);

		pt.x += f.second.x();
		pt.y += f.second.y();
		pt.z += f.second.z();

		m.points.push_back(pt);

		float mag = std::sqrt(f.second.x()*f.second.x() + f.second.y()*f.second.y() + f.second.z()*f.second.z())/0.0125;
		std_msgs::ColorRGBA color;
		color.a = 1.0;
		colormap(igl::inferno_cm, mag, color.r, color.g, color.b);

		m.colors.push_back(color);
		m.colors.push_back(color);
	}
	return m;
}


class Tracker
{
public:
	using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
	using DepthTraits = depth_image_proc::DepthTraits<uint16_t>;
	using Vector = Eigen::Vector3d;
	using Flow3D = std::vector<std::pair<Vector, Vector>>;
	using Flow2D = std::vector<std::pair<cv::Point2f, cv::Point2f>>;

	struct SubscriptionOptions
	{
		ros::NodeHandle nh, pnh;
//		image_transport::ImageTransport it;
		image_transport::TransportHints hints;
		int buffer;
		std::string topic_prefix;
		std::string rgb_topic;
		std::string depth_topic;
		std::string cam_topic;
		SubscriptionOptions(const std::string& prefix = "/kinect2_victor_head/hd")
			: nh(), pnh("~"),
//			  it(nh),
			  hints("compressed", ros::TransportHints(), pnh),
			  buffer(10), topic_prefix(prefix),
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
		ROI roi;

		TrackingOptions() : featureRadius(250.0f), pixelRadius(30.0f), meterRadius(0.05),
		                    roi({-0.4f, -0.6f, -0.020f}, {0.4f, 0.6f, 0.4f}, "table_surface")
		{
		}
	};

	const size_t MAX_BUFFER_LEN;//1000;

	std::vector<cv_bridge::CvImagePtr> rgb_buffer;
	std::vector<cv_bridge::CvImagePtr> depth_buffer;
	image_geometry::PinholeCameraModel cameraModel;

	std::unique_ptr<image_transport::ImageTransport> it;
	std::unique_ptr<image_transport::SubscriberFilter> rgb_sub;
	std::unique_ptr<image_transport::SubscriberFilter> depth_sub;
	std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> cam_sub;
	std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync;

	cv::Mat mask;
	std::vector<Flow3D> flows3;
	std::vector<Flow2D> flows2;

	SubscriptionOptions options;
	TrackingOptions track_options;

	std::shared_ptr<tf::TransformListener> listener;
	ros::Publisher vizPub;

	ros::CallbackQueue callback_queue;
	ros::AsyncSpinner spinner;

	explicit
	Tracker(const size_t _buffer = 500,
		SubscriptionOptions _options = SubscriptionOptions(),
		TrackingOptions _track_options = TrackingOptions());

	void startCapture();

	void stopCapture();

	void reset();

	cv::Mat& getMask();

	virtual
	void track(const size_t step = 1);

	void imageCb(const sensor_msgs::ImageConstPtr& rgb_msg,
	             const sensor_msgs::ImageConstPtr& depth_msg,
	             const sensor_msgs::CameraInfoConstPtr& cam_msg);
};

struct CaptureGuard
{
public:
	Tracker* tracker;
	CaptureGuard(Tracker* t) : tracker(t) {}
	~CaptureGuard() { if (tracker) { tracker->stopCapture(); } }
};

/// \brief Estimates the rigid transformation
/// \param flow Points 'a' match to 'b'
/// \param bTa Transform that carries a to b
/// \return successful
bool estimateRigidTransform(const Tracker::Flow3D& flow, Eigen::Isometry3d& bTa);

#endif // MPS_VOXELS_TRACKER_H
