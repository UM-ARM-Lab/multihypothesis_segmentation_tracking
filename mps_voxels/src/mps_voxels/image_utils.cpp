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

#include "mps_voxels/image_utils.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/project_point.hpp"
//#include "mps_voxels/ROI.h"

#define CV_VERSION_AT_LEAST(x,y,z) (CV_VERSION_MAJOR>x || (CV_VERSION_MAJOR>=x && \
                                   (CV_VERSION_MINOR>y || (CV_VERSION_MINOR>=y && \
                                                           CV_VERSION_REVISION>=z))))

namespace mps
{
pcl::PointCloud<PointT>::Ptr
imagesToCloud(const cv::Mat& rgb, const cv::Mat& depth, const image_geometry::PinholeCameraModel& cameraModel)
{
	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	cloud->resize(cameraModel.cameraInfo().width * cameraModel.cameraInfo().height);

#pragma omp parallel for
	for (int v = 0; v < (int) cameraModel.cameraInfo().height; ++v)
	{
		for (int u = 0; u < (int) cameraModel.cameraInfo().width; ++u)
		{
			int idx = v * (int) cameraModel.cameraInfo().width + u;

			auto color = rgb.at<cv::Vec3b>(v, u);
			auto dVal = depth.at<uint16_t>(v, u);

			float depthVal = mps::SensorHistorian::DepthTraits::toMeters(
				dVal); // if (depth1 > maxZ || depth1 < minZ) { continue; }
			PointT pt(color[2], color[1], color[0]);
			pt.getVector3fMap() = toPoint3D<Eigen::Vector3f>(u, v, depthVal, cameraModel);
			cloud->points[idx] = pt;
		}
	}

	return cloud;
}

cv::Scalar randomColor(cv::RNG& rng)
{
	int icolor = (unsigned) rng;
	return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}

std::set<uint16_t> unique(const cv::Mat& input)
{
	if (input.channels() > 1 || input.type() != CV_16U)
	{
		throw std::logic_error("unique !!! Only works with CV_16U 1-channel Mat");
	}

	std::set<uint16_t> out;

	#pragma omp declare reduction (merge : std::set<uint16_t> : omp_out.insert(omp_in.begin(), omp_in.end()))
	#pragma omp parallel for reduction(merge: out)
	for (int y = 0; y < input.rows; ++y)
	{
		const uint16_t* row_ptr = input.ptr<uint16_t>(y);
		for (int x = 0; x < input.cols; ++x)
		{
			out.insert(row_ptr[x]);
		}
	}

	return out;
}

cv::Mat colorByLabel(const cv::Mat& input)
{
	double min;
	double max;
	cv::minMaxIdx(input, &min, &max);
	cv::Mat labels;
	if (max > 255)
	{
		cv::Mat mod;
		input.copyTo(mod);

		uchar depth = mod.type() & CV_MAT_DEPTH_MASK;

		switch (depth)
		{
		case CV_8U: mod.forEach<uint8_t>([&](uint8_t& label, const int*) -> void { label = label % 256; });
			break;
		case CV_8S: mod.forEach<int8_t>([&](int8_t& label, const int*) -> void { label = label % 256; });
			break;
		case CV_16U: mod.forEach<uint16_t>([&](uint16_t& label, const int*) -> void { label = label % 256; });
			break;
		case CV_16S: mod.forEach<int16_t>([&](int16_t& label, const int*) -> void { label = label % 256; });
			break;
		case CV_32S: mod.forEach<int32_t>([&](int32_t& label, const int*) -> void { label = label % 256; });
			break;
		default: throw std::runtime_error("Unknown image data type in colorByLabel: " + std::to_string(input.type()));
			break;
		}

		mod.convertTo(labels, CV_8UC1);
	}
	else
	{
		input.convertTo(labels, CV_8UC1);
	}


	cv::Mat output;
#if CV_VERSION_AT_LEAST(3, 3, 0)
	cv::Mat colormap(256, 1, CV_8UC3);
	cv::randu(colormap, 0, 256);
	cv::applyColorMap(labels, output, colormap);
#else
	if (max < 255)
	{
		cv::normalize(labels, labels, 255, 0, cv::NORM_MINMAX);
	}
	cv::applyColorMap(labels, output, cv::COLORMAP_HSV);
#endif

	return output;
}

cv::Mat colorByLabel(const cv::Mat& input, const Colormap& colormap)
{
	assert(input.type() == CV_16UC1);
	using ColorPixel = cv::Point3_<uint8_t>;
	cv::Mat output(input.size(), CV_8UC3);
	output.forEach<ColorPixel>([&](ColorPixel& px, const int* pos) -> void
	                           {
		                           uint16_t label = input.at<uint16_t>(pos[0], pos[1]);
		                           px = colormap.at(label);
//		                           auto iter = colormap.find(label);
//		                           if (iter != colormap.end())
//		                           {
//			                           px = iter->second;
//		                           }
	                           });

	return output;
}

Colormap createColormap(const std::set<uint16_t>& labels, std::mt19937& re)
{
	Colormap colormap;
	std::uniform_int_distribution<> dis(0, 256);
	for (auto label : labels)
	{
		colormap[label] = cv::Point3_<uint8_t>(dis(re), dis(re), dis(re));
	}
	return colormap;
}
Colormap createColormap(const cv::Mat& labels, std::mt19937& re)
{
	return createColormap(unique(labels), re);
}

void extendColormap(Colormap& colormap, const std::set<uint16_t>& labels, std::mt19937& re)
{
	std::uniform_int_distribution<> dis(0, 256);
	for (auto label : labels)
	{
		auto iter = colormap.find(label);
		if (iter == colormap.end())
		{
			colormap.emplace(label, cv::Point3_<uint8_t>(dis(re), dis(re), dis(re)));
		}
	}
}
void extendColormap(Colormap& colormap, const cv::Mat& labels, std::mt19937& re)
{
	extendColormap(colormap, unique(labels), re);
}

}
