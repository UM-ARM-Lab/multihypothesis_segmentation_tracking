//
// Created by arprice on 8/24/18.
//

#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/pointcloud_utils.h"
#include "mps_voxels/assert.h"

namespace mps
{

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
	si->frame_id = cam.header.frame_id;
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
	for (int u = 1; u < si->labels2.cols; u += 2)
	{
		for (int v = 1; v < si->labels2.rows; v += 2)
		{
			si->labels.at<uint16_t>(v / 2, u / 2) = si->labels2.at<uint16_t>(v, u);
		}
	}

	cv::Mat tempContours1;
	double maxVal;
	cv::minMaxLoc(si->ucm2, nullptr, &maxVal);
	si->ucm2.convertTo(tempContours1, CV_8UC1, 255.0 / maxVal);
	cv::applyColorMap(tempContours1, si->display_contours,
	                  cv::COLORMAP_BONE);//cv::COLORMAP_PARULA);//cv::COLORMAP_JET); // COLORMAP_HOT

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

CachingRGBDSegmenter::CachingRGBDSegmenter(ros::NodeHandle& nh)
	:RGBDSegmenter(nh)
{ }

std::map<ObjectIndex, pcl::PointCloud<PointT>::Ptr> segmentCloudsFromImage(
	const pcl::PointCloud<PointT>::Ptr& cloud, const cv::Mat& labels,
	const image_geometry::PinholeCameraModel& cameraModel, const cv::Rect& roi,
	std::map<uint16_t, ObjectIndex>* labelToIndexLookup)
{
	MPS_ASSERT(roi.width == labels.cols);
	MPS_ASSERT(roi.height == labels.rows);
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

		MPS_ASSERT(bounds.contains(imagePt));

		LabelT label = filtered_labels.at<LabelT>(imagePt);
		if (BUFFER_VALUE == label) { continue; }
		segment_clouds[label]->push_back(pt);
	}

	const int nNeighbors = 200;
	const double percentThreshold = 0.04;
	const int sizeThreshold = 25;

	for (auto& segment : segment_clouds)
	{
		if (segment.second->size() >= nNeighbors)
		{
			segment.second = filterOutliers(segment.second, nNeighbors, 2.0);
		}
	}

//	for (const auto label : uniqueLabels)
//	{
//		if (label < 100)
//		{
//			segment_clouds.erase(label);
//			break;
//		}
//	}

	std::map<ObjectIndex, pcl::PointCloud<PointT>::Ptr> retVal;
	for (auto& pair : segment_clouds)
	{
		double percentFilled = static_cast<double>(pair.second->size())
		                       / static_cast<double>(cv::countNonZero(filtered_labels == pair.first));
		if (percentFilled >= percentThreshold && static_cast<int>(pair.second->size()) > sizeThreshold)
		{
			ObjectIndex objID{-(static_cast<long>(retVal.size()) - 100)}; // retVal.size()-1
			retVal.insert({objID, pair.second});
			if (labelToIndexLookup)
			{
				auto res = labelToIndexLookup->insert({pair.first, objID});
				MPS_ASSERT(res.second);
			}
		}
		else { std::cerr << "Rejected object " << pair.first << ": " << percentFilled * 100.0 << "%." << std::endl; }
	}

	return retVal;
}

//std::map<uint16_t, std::vector<xycoor>> segmentMaskFromImage(const cv::Mat& labels,const cv::Rect& roi){
//	std::map<uint16_t, std::vector<xycoor>> labelToMaskLookup;
//	MPS_ASSERT(roi.width == labels.cols);
//	MPS_ASSERT(roi.height == labels.rows);
//	using LabelT = uint16_t;
//	const LabelT BUFFER_VALUE = std::numeric_limits<LabelT>::max();
//	std::set<LabelT> uniqueLabels = unique(labels);
//
//	std::map<LabelT, int> labelMap;
//	cv::Mat filtered_labels(labels.size(), labels.type(), BUFFER_VALUE);
//	for (const auto label : uniqueLabels)
//	{
//		labelMap.insert({label, (int)labelMap.size()});
//
//		// Erode the boundary of the label slightly
//		cv::Mat labelMask = (labels == label);
//		cv::erode(labelMask, labelMask, cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
//		filtered_labels.setTo(label, labelMask);
//	}
//	return labelToMaskLookup;
//}

std::map<uint16_t, mps_msgs::AABBox2d> getBBox(const cv::Mat& labels, const cv::Rect& roi)
{
	MPS_ASSERT(roi.width == labels.cols);
	MPS_ASSERT(roi.height == labels.rows);
	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup;
	using LabelT = uint16_t;
	std::set<LabelT> uniqueLabels = unique(labels);

	for (const auto label : uniqueLabels)
	{
		mps_msgs::AABBox2d bbox;
//		bbox.xmin = labels.cols;
//		bbox.xmax = 0;
//		bbox.ymin = labels.rows;
//		bbox.ymax = 0;
//		for(int i = 0;i < labels.rows;i++) {
//			auto row = labels.ptr<LabelT>(i);
////			auto row = labels.row(i);
//			if(std::find(row, row + labels.cols, label) != row + labels.cols){
//				bbox.ymin = i;
//				break;
//			}
//		}
//		for(int i = labels.rows-1;i >= 0;i--) {
//			auto row = labels.ptr<LabelT>(i);
//			if(std::find(row, row + labels.cols, label) != row + labels.cols){
//				bbox.ymax = i;
//				break;
//			}
//		}
//		cv::Mat labelMask = (labels == label);
		cv::Rect box = cv::boundingRect(labels == label);

		if (2 * box.height * box.width > roi.height * roi.width) { continue; } // get rid of table segment
		bbox.xmin = box.x + roi.x;
		bbox.xmax = box.x + box.width + roi.x;
		bbox.ymin = box.y + roi.y;
		bbox.ymax = box.y + box.height + roi.y;

//		labelMask.
//		cv::boundingRect();
		labelToBBoxLookup.insert({label, bbox});
	}
	return labelToBBoxLookup;
}

}