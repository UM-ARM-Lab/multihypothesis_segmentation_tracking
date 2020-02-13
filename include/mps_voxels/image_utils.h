//
// Created by arprice on 8/29/18.
//

#ifndef MPS_VOXELS_IMAGE_UTILS_H
#define MPS_VOXELS_IMAGE_UTILS_H

#include "mps_voxels/PointT.h"
#include "mps_voxels/util/assert.h"

#include <opencv2/core.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <set>
#include <random>

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
