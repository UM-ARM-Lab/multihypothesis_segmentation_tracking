//
// Created by arprice on 8/24/18.
//

#ifndef MPS_VOXELS_SEGMENTATION_UTILS_H
#define MPS_VOXELS_SEGMENTATION_UTILS_H

#include "mps_voxels/ROI.h"
#include "mps_voxels/PointT.h"
#include "mps_voxels/ObjectIndex.h"

#include <mps_msgs/SegmentRGBDAction.h>
#include <mps_msgs/AABBox2d.h>

#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <actionlib/client/simple_action_client.h>
#include <ros/ros.h>

struct xycoor{
	int x;
	int y;
};


struct SegmentationInfo
{
	ros::Time t;
	cv::Rect roi; ///< ROI in the parent image
	cv::Mat rgb;
	cv::Mat depth;
	cv::Mat ucm2; // NB: The UCM is 2x the resolution of the original image, so it goes between the pixels of the original
	cv::Mat labels2; ///< Labels corresponding to the UCM
	cv::Mat centroids2;
	cv::Mat stats2;
	cv::Mat display_contours;
	cv::Mat labels; ///< Labels2 downsampled to the original size
	cv_bridge::CvImagePtr objectness_segmentation;
};

class RGBDSegmenter
{
public:
	using LabelType = uint16_t;
	using SegmentationClient = actionlib::SimpleActionClient<mps_msgs::SegmentRGBDAction>;

	explicit
	RGBDSegmenter(ros::NodeHandle& nh);

	mutable
	SegmentationClient segmentClient;

	virtual
	std::shared_ptr<SegmentationInfo> segment(
		const cv_bridge::CvImage& rgb,
		const cv_bridge::CvImage& depth,
		const sensor_msgs::CameraInfo& cam) const;

};

class CachingRGBDSegmenter : public RGBDSegmenter
{
public:
	using SegmentationCache = std::map<ros::Time, std::shared_ptr<SegmentationInfo> >;

	mutable
	SegmentationCache cache;

	explicit
	CachingRGBDSegmenter(ros::NodeHandle& nh);

	std::shared_ptr<SegmentationInfo> segment(
		const cv_bridge::CvImage& rgb,
		const cv_bridge::CvImage& depth,
		const sensor_msgs::CameraInfo& cam) const override;
};

std::map<ObjectIndex, pcl::PointCloud<PointT>::Ptr> segmentCloudsFromImage(
	const pcl::PointCloud<PointT>::Ptr& cloud, const cv::Mat& labels,
	const image_geometry::PinholeCameraModel& cameraModel, const cv::Rect& roi,
	std::map<uint16_t, ObjectIndex>* labelToIndexLookup = nullptr);

//std::map<uint16_t, std::vector<xycoor>> segmentMaskFromImage(const cv::Mat& labels,const cv::Rect& roi);

std::map<uint16_t, mps_msgs::AABBox2d> getBBox(const cv::Mat& labels,const cv::Rect& roi);

#endif // MPS_VOXELS_SEGMENTATION_UTILS_H
