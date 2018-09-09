//
// Created by arprice on 8/24/18.
//

#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/pointcloud_utils.h"

#include <mps_msgs/SegmentRGBD.h>

RGBDSegmenter::RGBDSegmenter(ros::NodeHandle& nh)
{
	segmentClient = nh.serviceClient<mps_msgs::SegmentRGBD>("/segment_rgbd");
	if (!segmentClient.waitForExistence(ros::Duration(3)))
	{
		ROS_WARN("Segmentation server not connected.");
	}

}

cv_bridge::CvImagePtr
RGBDSegmenter::segment(const cv_bridge::CvImage& rgb, const cv_bridge::CvImage& depth,
                       const sensor_msgs::CameraInfo& cam) const
{
	mps_msgs::SegmentRGBDRequest request;
	mps_msgs::SegmentRGBDResponse response;

	rgb.toImageMsg(request.rgb);
	depth.toImageMsg(request.depth);
	request.camera_info = cam;

	if (segmentClient.exists())
	{
		if (segmentClient.call(request, response))
		{
			if (!response.segmentation.data.empty())
			{
				return cv_bridge::toCvCopy(response.segmentation, "mono16");
			}
		}
	}

	return cv_bridge::CvImagePtr(nullptr);
}


std::vector<pcl::PointCloud<PointT>::Ptr> segment(
	const pcl::PointCloud<PointT>::Ptr& cloud, const cv::Mat& labels,
	const image_geometry::PinholeCameraModel& cameraModel, const cv::Rect& roi)
{
	assert(roi.width == labels.cols);
	assert(roi.height == labels.rows);
	using LabelT = uint16_t;
	const LabelT BUFFER_VALUE = std::numeric_limits<LabelT>::max();
	std::set<LabelT> uniqueLabels = unique(labels);

	std::vector<pcl::PointCloud<PointT>::Ptr> segment_clouds;
	std::map<LabelT, int> labelMap;
	cv::Mat filtered_labels(labels.size(), labels.type(), BUFFER_VALUE);
	for (const auto label : uniqueLabels)
	{
		labelMap.insert({label, (int)labelMap.size()});
		segment_clouds.push_back(pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>()));

		// Erode the boundary of the label slightly
		cv::Mat labelMask = (labels == label);
		cv::erode(labelMask, labelMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
		filtered_labels.setTo(label, labelMask);
	}

	cv::Rect bounds(cv::Point(), labels.size());
	for (const PointT& pt : *cloud)
	{
		// NB: cloud is in the camera frame
		Eigen::Vector3d p = pt.getVector3fMap().cast<double>();
		cv::Point3d worldPt_camera(p.x(), p.y(), p.z());
		cv::Point2d imagePt = cameraModel.project3dToPixel(worldPt_camera);
		imagePt.x -= roi.x;
		imagePt.y -= roi.y;

		if (bounds.contains(imagePt))
		{
			LabelT label = filtered_labels.at<LabelT>(imagePt);
			if (BUFFER_VALUE == label) { continue; }
			segment_clouds[labelMap.at(label)]->push_back(pt);
		}
		else
		{
			throw std::runtime_error("Point did not reproject onto label mask.");
		}
	}

	const int threshold = 100;

	for (auto& segment : segment_clouds)
	{
		if (segment->size() >= threshold)
		{
			segment = filterOutliers(segment, threshold);
		}
	}

	for (const auto label : uniqueLabels)
	{
		if (label < 100)
		{
			segment_clouds.erase(segment_clouds.begin() + labelMap.at(label));
			break;
		}
	}

	segment_clouds.erase(std::remove_if(segment_clouds.begin(), segment_clouds.end(),
	                                  [](const pcl::PointCloud<PointT>::Ptr& pc){ return pc->size() < threshold;}),
	                     segment_clouds.end());

	return segment_clouds;
}