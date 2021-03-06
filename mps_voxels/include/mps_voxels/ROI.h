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

#ifndef MPS_VOXELS_ROI_H
#define MPS_VOXELS_ROI_H

//#include <Eigen/Geometry>

#include <tf/transform_datatypes.h>
#include <image_geometry/pinhole_camera_model.h>
#include <depth_image_proc/depth_conversions.h>
#include <visualization_msgs/MarkerArray.h>

namespace mps
{

template <typename PointT>
std::vector<PointT> getCorners(const PointT& min, const PointT& max, const int dimension)
{
	// There will be 2^DIM corners to deal with
	const unsigned nCorners = (1u << dimension);
	std::vector<PointT> corners(nCorners);
	for (unsigned perm = 0; perm < nCorners; ++perm)
	{
		PointT pt;
		for (int d = 0; d < dimension; ++d)
		{
			pt[d] = (perm & (1u<<d)) ? min[d] : max[d];
		}
		corners[perm] = pt;
	}
	return corners;
}


template <typename PointT>
visualization_msgs::Marker visualizeAABB(const PointT& minExtent, const PointT& maxExtent)
{
	visualization_msgs::Marker m;
	m.type = visualization_msgs::Marker::CUBE;
	m.action = visualization_msgs::Marker::ADD;
	m.frame_locked = true;
	m.header.frame_id = "table_surface";
	m.header.stamp = ros::Time::now();
	m.pose.orientation.w = 1.0f;
	m.pose.position.x = (minExtent.x() + maxExtent.x())/2.0f;
	m.pose.position.y = (minExtent.y() + maxExtent.y())/2.0f;
	m.pose.position.z = (minExtent.z() + maxExtent.z())/2.0f;
	m.scale.x = (maxExtent.x() - minExtent.x());
	m.scale.y = (maxExtent.y() - minExtent.y());
	m.scale.z = (maxExtent.z() - minExtent.z());
	m.color.a = 0.5;
	m.color.b = 1.0;
	m.id = 0;
	m.ns = "aabb";
	return m;
}


class ROI
{
public:
	using DepthTraits = depth_image_proc::DepthTraits<uint16_t>;
	using PointT = tf::Vector3;

	PointT minExtent;
	PointT maxExtent;
	std::string frame_id;

	ROI(const PointT& min = PointT{-0.4f, -0.6f, -0.020f},
		const PointT& max = PointT{0.4f, 0.6f, 0.4f},
		const std::string& frame = "table_surface")
		: minExtent(min), maxExtent(max), frame_id(frame) {}

	cv::Mat getMask(const tf::StampedTransform& cameraTworld,
	                const image_geometry::PinholeCameraModel& cam,
	                const cv::Mat* depth = nullptr) const;
	visualization_msgs::Marker getMarker() const;
};

}

#endif // MPS_VOXELS_ROI_H
