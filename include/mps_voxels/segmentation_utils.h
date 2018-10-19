//
// Created by arprice on 8/24/18.
//

#ifndef MPS_VOXELS_SEGMENTATION_UTILS_H
#define MPS_VOXELS_SEGMENTATION_UTILS_H

#include "mps_voxels/ROI.h"
#include "mps_voxels/PointT.h"

#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <ros/ros.h>

class RGBDSegmenter
{
public:
	using LabelType = uint16_t;
	explicit
	RGBDSegmenter(ros::NodeHandle& nh);

	mutable
	ros::ServiceClient segmentClient;

	cv_bridge::CvImagePtr segment(const cv_bridge::CvImage& rgb, const cv_bridge::CvImage& depth,
	                              const sensor_msgs::CameraInfo& cam, cv_bridge::CvImagePtr* contours = nullptr) const;

};

std::vector<pcl::PointCloud<PointT>::Ptr> segment(
	const pcl::PointCloud<PointT>::Ptr& cloud, const cv::Mat& labels,
	const image_geometry::PinholeCameraModel& cameraModel, const cv::Rect& roi,
	std::map<uint16_t, int>* labelToIndexLookup = nullptr);

#endif // MPS_VOXELS_SEGMENTATION_UTILS_H
