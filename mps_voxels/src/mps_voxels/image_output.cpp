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

#include "mps_voxels/image_output.h"
#include "mps_voxels/logging/log_cv_mat.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/node_handle.h>

namespace mps
{

class ImageOutputImpl
{
public:
	ros::NodeHandle pnh;
	image_transport::ImageTransport it;

	ImageOutputImpl()
		: pnh("~"),
		  it(pnh)
	{
	};

	image_transport::Publisher* findOrCreate(const std::string& channel)
	{
		auto iter = imagePubs.find(channel);
		if (iter == imagePubs.end())
		{
			auto res = imagePubs.emplace(channel, std::make_unique<image_transport::Publisher>(it.advertise(channel, 1, true)));
			sleep(2);
			return res.first->second.get();
		}
		else
		{
			return iter->second.get();
		}
	}

	void initChannel(const std::string& channel)
	{
		findOrCreate(channel);
	}

	void sendImage(const std::string& channel, const cv::Mat& img)
	{
		const auto* pub = findOrCreate(channel);
		std_msgs::Header header;
		header.stamp = ros::Time::now();
		header.frame_id = "";
		if (pub->getNumSubscribers() > 0)
		{
			pub->publish(cv_bridge::CvImage(header, cvType2Str(img.type()), img).toImageMsg());
		}
	}

protected:
	std::map<std::string, std::unique_ptr<image_transport::Publisher>> imagePubs;
};

ImageOutputSingleton::ImageOutputSingleton() : sImpl(std::make_unique<ImageOutputImpl>())
{
}


ImageOutputSingleton& ImageOutputSingleton::instance()
{
	static ImageOutputSingleton sInstance;
	return sInstance;
}

void ImageOutputSingleton::initChannel(const std::string& channel)
{
	instance().sImpl->initChannel(channel);
}

void ImageOutputSingleton::sendImage(const std::string& channel, const cv::Mat& img)
{
	instance().sImpl->sendImage(channel, img);
}

}
