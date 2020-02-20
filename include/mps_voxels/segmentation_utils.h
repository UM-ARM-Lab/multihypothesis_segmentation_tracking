//
// Created by arprice on 8/24/18.
//

#ifndef MPS_VOXELS_SEGMENTATION_UTILS_H
#define MPS_VOXELS_SEGMENTATION_UTILS_H

#include "mps_voxels/SegmentationInfo.h"
#include "mps_voxels/ROI.h"
#include "mps_voxels/PointT.h"
#include "mps_voxels/Indexes.h"

#include <mps_msgs/SegmentRGBDAction.h>
#include <mps_msgs/AABBox2d.h>

#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

#include <boost/bimap.hpp>

namespace mps
{

class RGBDSegmenter
{
public:
	using LabelType = uint16_t;
	using SegmentationClient = actionlib::SimpleActionClient<mps_msgs::SegmentRGBDAction>;

	explicit
	RGBDSegmenter(ros::NodeHandle &nh);

	mutable
	SegmentationClient segmentClient;

	virtual
	std::shared_ptr<SegmentationInfo> segment(
		const cv_bridge::CvImage &rgb,
		const cv_bridge::CvImage &depth,
		const sensor_msgs::CameraInfo &cam) const;

};

class CachingRGBDSegmenter : public RGBDSegmenter
{
public:
	using SegmentationCache = std::map<ros::Time, std::shared_ptr<SegmentationInfo>>;

	mutable
	SegmentationCache cache;

	explicit
	CachingRGBDSegmenter(ros::NodeHandle &nh);

	std::shared_ptr<SegmentationInfo> segment(
		const cv_bridge::CvImage &rgb,
		const cv_bridge::CvImage &depth,
		const sensor_msgs::CameraInfo &cam) const override;
};

std::map<ObjectIndex, pcl::PointCloud<PointT>::Ptr> segmentCloudsFromImage(
	const pcl::PointCloud<PointT>::Ptr &cloud, const cv::Mat &labels,
	const image_geometry::PinholeCameraModel &cameraModel, const cv::Rect &roi,
	boost::bimap<uint16_t, ObjectIndex> *labelToIndexLookup = nullptr);

std::map<uint16_t, mps_msgs::AABBox2d> getBBox(const cv::Mat &labels, const cv::Rect& roi = {0, 0, 1920,1080}, const int& dilation = 0);

pcl::PointCloud<PointT>::Ptr make_PC_segment(const cv::Mat &rgb, const cv::Mat &depth,
                                             const image_geometry::PinholeCameraModel &cameraModel,
                                             const cv::Mat &mask);

}
#endif // MPS_VOXELS_SEGMENTATION_UTILS_H
