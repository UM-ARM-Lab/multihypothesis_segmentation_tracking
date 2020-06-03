/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
