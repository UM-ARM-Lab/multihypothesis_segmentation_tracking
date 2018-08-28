//
// Created by arprice on 8/24/18.
//

#include "mps_voxels/segmentation_utils.h"
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
				return cv_bridge::toCvCopy(response.segmentation);
			}
		}
	}

	return cv_bridge::CvImagePtr(nullptr);
}