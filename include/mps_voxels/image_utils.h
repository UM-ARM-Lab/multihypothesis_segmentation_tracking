//
// Created by arprice on 8/29/18.
//

#ifndef MPS_VOXELS_IMAGE_UTILS_H
#define MPS_VOXELS_IMAGE_UTILS_H

#include "mps_voxels/PointT.h"

#include <opencv2/core.hpp>
#include <image_geometry/pinhole_camera_model.h>

#include <set>

pcl::PointCloud<PointT>::Ptr imagesToCloud(const cv::Mat& rgb, const cv::Mat& depth, const image_geometry::PinholeCameraModel& cameraModel);

cv::Scalar randomColor( cv::RNG& rng );

std::set<uint16_t> unique(const cv::Mat& input);

cv::Mat colorByLabel(const cv::Mat& input);

#endif // MPS_VOXELS_IMAGE_UTILS_H
