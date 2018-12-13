//
// Created by arprice on 8/24/18.
//

#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/pointcloud_utils.h"

RGBDSegmenter::RGBDSegmenter(ros::NodeHandle& nh)
	: segmentClient(nh, "/segment_rgbd", true)
{
	if (!segmentClient.waitForServer(ros::Duration(3)))
	{
		ROS_WARN("Segmentation server not connected.");
	}

}

std::shared_ptr<SegmentationInfo>
RGBDSegmenter::segment(const cv_bridge::CvImage& rgb, const cv_bridge::CvImage& depth,
                       const sensor_msgs::CameraInfo& cam) const
{
	mps_msgs::SegmentRGBDGoal request;
	mps_msgs::SegmentRGBDResultConstPtr response;

	rgb.toImageMsg(request.rgb);
	depth.toImageMsg(request.depth);
	request.camera_info = cam;

	if (!segmentClient.isServerConnected())
	{
		return std::shared_ptr<SegmentationInfo>();
	}

	auto success = segmentClient.sendGoalAndWait(request);
	if (!success.isDone())
	{
		return std::shared_ptr<SegmentationInfo>();
	}

	response = segmentClient.getResult();
	if (response->segmentation.data.empty())
	{
		return std::shared_ptr<SegmentationInfo>();
	}

	std::shared_ptr<SegmentationInfo> si = std::make_shared<SegmentationInfo>();
	si->t = cam.header.stamp;
	si->rgb = rgb.image;
	si->depth = depth.image;
	si->roi.width = cam.roi.width;
	si->roi.height = cam.roi.height;
	si->roi.x = cam.roi.x_offset;
	si->roi.y = cam.roi.y_offset;

	si->objectness_segmentation = cv_bridge::toCvCopy(response->segmentation, "mono16");
	// NB: ucm2 comes back at twice the resolution of the original image
	si->ucm2 = cv_bridge::toCvCopy(response->contours, "64FC1")->image;

	cv::connectedComponentsWithStats(si->ucm2 == 0, si->labels2, si->stats2, si->centroids2, 8, CV_16U);
	si->labels = cv::Mat(si->rgb.size(), CV_16U);
	for (int u = 1; u < si->labels2.cols; u+=2)
	{
		for (int v = 1; v < si->labels2.rows; v+=2)
		{
			si->labels.at<uint16_t>(v/2, u/2) = si->labels2.at<uint16_t>(v, u);
		}
	}

	cv::Mat tempContours1;
	double maxVal;
	cv::minMaxLoc(si->ucm2, nullptr, &maxVal);
	si->ucm2.convertTo(tempContours1, CV_8UC1, 255.0/maxVal);
	cv::applyColorMap(tempContours1, si->display_contours, cv::COLORMAP_BONE);//cv::COLORMAP_PARULA);//cv::COLORMAP_JET); // COLORMAP_HOT

	return si;
}

std::shared_ptr<SegmentationInfo>
CachingRGBDSegmenter::segment(const cv_bridge::CvImage& rgb, const cv_bridge::CvImage& depth,
                              const sensor_msgs::CameraInfo& cam) const
{
	const ros::Time& t = cam.header.stamp;
	auto si_iter = cache.find(t);
	if (si_iter != cache.end())
	{
		return si_iter->second;
	}

	std::shared_ptr<SegmentationInfo> si = RGBDSegmenter::segment(rgb, depth, cam);

	cache.insert({t, si});

	return si;
}

CachingRGBDSegmenter::CachingRGBDSegmenter(ros::NodeHandle& nh) : RGBDSegmenter(nh) {}


std::vector<pcl::PointCloud<PointT>::Ptr> segmentCloudsFromImage(
	const pcl::PointCloud<PointT>::Ptr& cloud, const cv::Mat& labels,
	const image_geometry::PinholeCameraModel& cameraModel, const cv::Rect& roi,
	std::map<uint16_t, int>* labelToIndexLookup)
{
	assert(roi.width == labels.cols);
	assert(roi.height == labels.rows);
	using LabelT = uint16_t;
	const LabelT BUFFER_VALUE = std::numeric_limits<LabelT>::max();
	std::set<LabelT> uniqueLabels = unique(labels);

	std::map<LabelT, pcl::PointCloud<PointT>::Ptr> segment_clouds;
//	std::map<LabelT, int> labelMap;
	cv::Mat filtered_labels(labels.size(), labels.type(), BUFFER_VALUE);
	for (const auto label : uniqueLabels)
	{
//		labelMap.insert({label, (int)labelMap.size()});
		segment_clouds.insert({label, pcl::PointCloud<PointT>::Ptr(new pcl::PointCloud<PointT>())});

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
			segment_clouds[label]->push_back(pt);
		}
		else
		{
			throw std::runtime_error("Point did not reproject onto label mask.");
		}
	}

	const int threshold = 100;

	for (auto& segment : segment_clouds)
	{
		if (segment.second->size() >= threshold)
		{
			segment.second = filterOutliers(segment.second, threshold);
		}
	}

	for (const auto label : uniqueLabels)
	{
		if (label < 100)
		{
			segment_clouds.erase(label);
			break;
		}
	}

	std::vector<pcl::PointCloud<PointT>::Ptr> retVal;
	retVal.reserve(segment_clouds.size());
	for (auto& pair : segment_clouds)
	{
		if (pair.second->size() >= threshold)
		{
			retVal.push_back(pair.second);
			if (labelToIndexLookup)
			{
				(*labelToIndexLookup)[pair.first] = retVal.size()-1;
			}
		}
	}

	return retVal;
}