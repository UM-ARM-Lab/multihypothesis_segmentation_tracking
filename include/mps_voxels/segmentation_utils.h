//
// Created by arprice on 8/24/18.
//

#ifndef MPS_VOXELS_SEGMENTATION_UTILS_H
#define MPS_VOXELS_SEGMENTATION_UTILS_H

#include "mps_voxels/ROI.h"

#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

class RGBDSegmenter
{
public:
	explicit
	RGBDSegmenter(ros::NodeHandle& nh);

	mutable
	ros::ServiceClient segmentClient;

	cv_bridge::CvImagePtr segment(const cv_bridge::CvImage& rgb, const cv_bridge::CvImage& depth,
	                              const sensor_msgs::CameraInfo& cam) const;

};

#endif // MPS_VOXELS_SEGMENTATION_UTILS_H
