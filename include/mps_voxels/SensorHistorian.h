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

#ifndef SRC_SENSORHISTORIAN_H
#define SRC_SENSORHISTORIAN_H

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <depth_image_proc/depth_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/JointState.h>

#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

namespace mps
{

/**
 * @class SensorHistoryBuffer
 * @brief Data structure for recording raw sensor output over a session of interest, possibly with sensor model info
 */
struct SensorHistoryBuffer
{
	image_geometry::PinholeCameraModel cameraModel;
	std::map<ros::Time, cv_bridge::CvImagePtr> rgb;
	std::map<ros::Time, cv_bridge::CvImagePtr> depth;
	std::map<ros::Time, sensor_msgs::JointStateConstPtr> joint;
	// We might want TFs here at some point
};

class SensorHistorian
{
public:
	using SyncPolicy = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
	using DepthTraits = depth_image_proc::DepthTraits<uint16_t>;

	struct SubscriptionOptions
	{
		ros::NodeHandle nh, pnh;
//		image_transport::ImageTransport it;
		image_transport::TransportHints hints;
		int queue_size;
		std::string topic_prefix;
		std::string rgb_topic;
		std::string depth_topic;
		std::string cam_topic;
		std::string joint_topic;
		SubscriptionOptions(const std::string& prefix = "/kinect2_victor_head/hd")
			:nh(), pnh("~"),
//			  it(nh),
              hints("compressed", ros::TransportHints(), pnh),
             queue_size(10), topic_prefix(prefix),
             rgb_topic(topic_prefix+"/image_color_rect"),
             depth_topic(topic_prefix+"/image_depth_rect"),
             cam_topic(topic_prefix+"/camera_info"),
             joint_topic("joint_states")
		{
		}
	};

	std::unique_ptr<image_transport::ImageTransport> it;
	std::unique_ptr<image_transport::SubscriberFilter> rgb_sub;
	std::unique_ptr<image_transport::SubscriberFilter> depth_sub;
	std::unique_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo>> cam_sub;
	std::unique_ptr<message_filters::Synchronizer<SyncPolicy>> sync;
	std::unique_ptr<ros::Subscriber> joint_sub;

	SubscriptionOptions options;
	const size_t MAX_BUFFER_LEN;//1000;

	tf::TransformListener* listener; // TODO: Shared ptr?

	ros::CallbackQueue callback_queue;
	ros::AsyncSpinner spinner;


	SensorHistoryBuffer buffer;

	explicit
	SensorHistorian(tf::TransformListener* _listener,
	                const size_t _buffer = 500,
	                SubscriptionOptions _options = SubscriptionOptions());

	~SensorHistorian();

	void startCapture();

	void stopCapture();

	void reset();

	void imageCb(const sensor_msgs::ImageConstPtr& rgb_msg,
	             const sensor_msgs::ImageConstPtr& depth_msg,
	             const sensor_msgs::CameraInfoConstPtr& cam_msg);

	void jointCb(const sensor_msgs::JointStateConstPtr& joint_msg);

	// TODO: TF Callback
};

struct CaptureGuard
{
public:
	SensorHistorian* tracker;

	CaptureGuard(SensorHistorian* t)
		:tracker(t)
	{ }

	~CaptureGuard()
	{ if (tracker) { tracker->stopCapture(); }}
};

}

#endif // SRC_SENSORHISTORIAN_H
