//
// Created by arprice on 8/22/18.
//

#include "mps_voxels/ROI.h"

namespace mps
{

cv::Mat ROI::getMask(const tf::StampedTransform& cameraTworld,
                     const image_geometry::PinholeCameraModel& cam,
                     const cv::Mat* depth) const
{
	cv::Mat mask = cv::Mat::zeros(cam.cameraInfo().height, cam.cameraInfo().width, CV_8UC1);

	std::vector<cv::Point2i> boxCorners;
	double minZ = std::numeric_limits<double>::max();
	double maxZ = std::numeric_limits<double>::lowest();
	for (const tf::Vector3& pt_world : getCorners(minExtent, maxExtent, 3))
	{
		tf::Vector3 pt_camera = cameraTworld * pt_world;
		minZ = std::min(minZ, pt_camera.z());
		maxZ = std::max(maxZ, pt_camera.z());
		cv::Point2d imgPt = cam.project3dToPixel({pt_camera.x(), pt_camera.y(), pt_camera.z()});
		boxCorners.push_back(imgPt);
	}
	std::vector<cv::Point2i> hullPoly;
	cv::convexHull(boxCorners, hullPoly);

	cv::fillConvexPoly(mask, hullPoly, cv::Scalar::all(255));

	if (depth)
	{
		cv::Mat depthMask = ((*depth) > DepthTraits::fromMeters(minZ))
		                    & ((*depth) < DepthTraits::fromMeters(maxZ));
		cv::dilate(depthMask, depthMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25,25)));
		cv::erode(depthMask, depthMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25,25)));
		cv::erode(depthMask, depthMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25,25)));
		cv::dilate(depthMask, depthMask, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(25,25)));

		mask &= depthMask;

//		cv::imshow("prev", depthMask);
//		cv::waitKey(100);
	}

//	cv::imshow("prev", mask);
//	cv::waitKey(100);

	return mask;
}

visualization_msgs::Marker ROI::getMarker() const
{
	visualization_msgs::Marker m = visualizeAABB(minExtent, maxExtent);
	m.header.frame_id = this->frame_id;
	return m;
}

}
