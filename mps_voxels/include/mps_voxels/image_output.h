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

#ifndef MPS_IMAGE_OUTPUT_H
#define MPS_IMAGE_OUTPUT_H

#include <opencv2/core.hpp>
#include <string>
#include <memory>

#define IMAGES_NONE 0
#define IMAGES_ROS  1
#define IMAGES_OCV  2

#ifndef IMAGE_VIEW_MODE
#define IMAGE_VIEW_MODE IMAGES_ROS
#endif

#define _unused(x) ((void)(x))

#if IMAGE_VIEW_MODE == IMAGES_OCV
#include <opencv2/highgui.hpp>
#endif

#if IMAGE_VIEW_MODE == IMAGES_NONE
#define WAIT_KEY(x) _unused((x))
#define IMSHOW(n, t) _unused((n)); _unused((t))
#define NAMED_WINDOW(n, t) _unused((n)); _unused((t))
#elif IMAGE_VIEW_MODE == IMAGES_ROS
#define WAIT_KEY(x) usleep((x)*1000);
#define IMSHOW(n, t) ::mps::ImageOutputSingleton::sendImage((n), (t))
#define NAMED_WINDOW(n, t) ::mps::ImageOutputSingleton::initChannel((n)); _unused((t))
#else
#define WAIT_KEY(x) cv::waitKey((x))
#define IMSHOW(n, t) cv::imshow((n), (t))
#define NAMED_WINDOW(n, t) cv::namedWindow((n), (t))
#endif


namespace mps
{

class ImageOutputImpl;

class ImageOutputSingleton
{
public:
	static ImageOutputSingleton& instance();
	static void initChannel(const std::string& channel);
	static void sendImage(const std::string& channel, const cv::Mat& img);
private:
	ImageOutputSingleton();

	std::unique_ptr<ImageOutputImpl> sImpl;
public:
	ImageOutputSingleton(const ImageOutputSingleton&) = delete;
	void operator=(const ImageOutputSingleton&)  = delete;
	~ImageOutputSingleton() = default;

};

}

#endif // MPS_IMAGE_OUTPUT_H
