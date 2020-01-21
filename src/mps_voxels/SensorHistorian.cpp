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

#include "mps_voxels/SensorHistorian.h"

namespace mps
{

SensorHistorian::SensorHistorian(tf::TransformListener* _listener, const size_t _buffer, SubscriptionOptions _options)
	: MAX_BUFFER_LEN(_buffer), options(std::move(_options)),
	  listener(_listener),
	  callback_queue(),
	  spinner(1, &callback_queue)
{
	options.nh.setCallbackQueue(&callback_queue);
	options.pnh.setCallbackQueue(&callback_queue);

	it = std::make_unique<image_transport::ImageTransport>(options.nh);
	rgb_sub = std::make_unique<image_transport::SubscriberFilter>(*it, options.rgb_topic, options.queue_size, options.hints);
	depth_sub = std::make_unique<image_transport::SubscriberFilter>(*it, options.depth_topic, options.queue_size, options.hints);
	cam_sub = std::make_unique<message_filters::Subscriber<sensor_msgs::CameraInfo>>(options.nh, options.cam_topic, options.queue_size);

	sync = std::make_unique<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(options.queue_size), *rgb_sub, *depth_sub, *cam_sub);
	sync->registerCallback(boost::bind(&SensorHistorian::imageCb, this, _1, _2, _3));

	spinner.start();
}

SensorHistorian::~SensorHistorian()
{
	stopCapture();
	reset();

	options.nh.shutdown();
	options.pnh.shutdown();

	spinner.stop();
	callback_queue.disable();
	callback_queue.clear();
}

void SensorHistorian::startCapture()
{
	reset();

	auto joint_sub_options = ros::SubscribeOptions::create<sensor_msgs::JointState>(options.joint_topic, 2, boost::bind(&SensorHistorian::jointCb, this, _1), ros::VoidPtr(), &callback_queue);
	joint_sub = std::make_unique<ros::Subscriber>(options.nh.subscribe(joint_sub_options));

	rgb_sub->subscribe(*it, options.rgb_topic, options.queue_size, options.hints);
	depth_sub->subscribe(*it, options.depth_topic, options.queue_size, options.hints);
	cam_sub->subscribe(options.nh, options.cam_topic, options.queue_size);
}

void SensorHistorian::stopCapture()
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

void SensorHistorian::reset()
{
	buffer.rgb.clear();
	buffer.depth.clear();
	buffer.joint.clear();
}

void SensorHistorian::imageCb(const sensor_msgs::ImageConstPtr& rgb_msg,
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

	buffer.cameraModel.fromCameraInfo(*cam_msg);

	if (buffer.rgb.size() < MAX_BUFFER_LEN)
	{
		buffer.rgb.insert({cv_rgb_ptr->header.stamp, cv_rgb_ptr});
		std::cerr << buffer.rgb.size() << ": " << buffer.rgb.rbegin()->first - buffer.rgb.begin()->first << std::endl;
	}

	if (buffer.depth.size() < MAX_BUFFER_LEN)
	{
		buffer.depth.insert({cv_depth_ptr->header.stamp, cv_depth_ptr});
	}

	if (buffer.rgb.size() == MAX_BUFFER_LEN)
	{
		stopCapture();
	}
}

void SensorHistorian::jointCb(const sensor_msgs::JointStateConstPtr& joint_msg)
{
	buffer.joint.insert(buffer.joint.end(), {joint_msg->header.stamp, joint_msg});
}

}
