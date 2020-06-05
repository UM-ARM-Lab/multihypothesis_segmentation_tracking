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

#ifndef MPS_VOXELS_IMAGE_UTILS_H
#define MPS_VOXELS_IMAGE_UTILS_H

#include "mps_voxels/PointT.h"
#include "mps_voxels/util/assert.h"

#include <opencv2/core.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <set>
#include <random>

#ifndef CV_OVERRIDE
#  define CV_OVERRIDE override
#endif

namespace mps
{

pcl::PointCloud<PointT>::Ptr
imagesToCloud(const cv::Mat& rgb, const cv::Mat& depth, const image_geometry::PinholeCameraModel& cameraModel);

cv::Scalar randomColor(cv::RNG& rng);

std::set<uint16_t> unique(const cv::Mat& input);

using Colormap = std::map<uint16_t, cv::Point3_<uint8_t>>;

cv::Mat colorByLabel(const cv::Mat& input);

cv::Mat colorByLabel(const cv::Mat& input, const Colormap& colormap);

Colormap createColormap(const std::set<uint16_t>& labels, std::default_random_engine& re);
Colormap createColormap(const cv::Mat& labels, std::default_random_engine& re);

void extendColormap(Colormap& colormap, const std::set<uint16_t>& labels, std::default_random_engine& re);
void extendColormap(Colormap& colormap, const cv::Mat& labels, std::default_random_engine& re);

template<typename Map>
cv::Mat relabel(const cv::Mat& input, const Map& map)
{
	cv::Mat output = cv::Mat::zeros(input.size(), CV_16UC1);

	output.forEach<uint16_t>([&](uint16_t& px, const int* pos) -> void
	                         {
		                         uint16_t label = input.at<uint16_t>(pos[0], pos[1]);
		                         auto iter = map.find(label);
		                         if (iter != map.end())
		                         {
			                         px = iter->second;
		                         }
	                         });

	return output;
}

}

#endif // MPS_VOXELS_IMAGE_UTILS_H
