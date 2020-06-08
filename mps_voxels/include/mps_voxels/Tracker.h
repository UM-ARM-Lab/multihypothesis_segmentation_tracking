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

#ifndef MPS_VOXELS_TRACKER_H
#define MPS_VOXELS_TRACKER_H

#include "mps_voxels/ROI.h"
#include "mps_voxels/colormap.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/util/assert.h"

#include "mps_msgs/AABBox2d.h"

#include <opencv2/imgproc.hpp>

#include <visualization_msgs/MarkerArray.h>
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

#include <Eigen/Geometry>

namespace mps
{

void drawKeypoint(cv::Mat& display, const cv::KeyPoint& kp, const cv::Scalar& color);

template<typename PointT>
visualization_msgs::Marker visualizeFlow(const std::vector<std::pair<PointT, PointT>>& flow)
{
	visualization_msgs::Marker m;
	m.type = visualization_msgs::Marker::LINE_LIST;
	m.action = visualization_msgs::Marker::ADD;
	m.frame_locked = true;
	m.header.frame_id = "kinect2_victor_head_rgb_optical_frame"; // TODO: This is different between robot and simulation
	m.header.stamp = ros::Time::now();
	m.pose.orientation.w = 1.0f;
	m.scale.x = 0.002;
//	m.scale.y = 1;
//	m.scale.z = 1;
	m.color.a = 1.0;
	m.color.r = 1.0;
	m.id = 0;
	m.ns = "flow";
	for (const auto& f : flow)
	{
		geometry_msgs::Point pt;
		pt.x = f.first.x();
		pt.y = f.first.y();
		pt.z = f.first.z();

		m.points.push_back(pt);

		pt.x += f.second.x();
		pt.y += f.second.y();
		pt.z += f.second.z();

		m.points.push_back(pt);

		float mag = std::sqrt(f.second.x() * f.second.x() + f.second.y() * f.second.y() + f.second.z() * f.second.z())
		            / 0.0125;
		std_msgs::ColorRGBA color;
		color.a = 1.0;
		colormap(igl::inferno_cm, mag, color.r, color.g, color.b);

		m.colors.push_back(color);
		m.colors.push_back(color);
	}
	return m;
}

class Tracker
{
public:
	using DepthTraits = depth_image_proc::DepthTraits<uint16_t>;
	using Vector = Eigen::Vector3d;
	using Flow3D = std::vector<std::pair<Vector, Vector>>;
	using Flow2D = std::vector<std::pair<cv::Point2f, cv::Point2f>>;

	struct TrackingOptions
	{
		float featureRadius;
		float pixelRadius;
		float meterRadius;
		ROI roi;
		std::string directory = "";

		TrackingOptions()
			:featureRadius(250.0f), pixelRadius(30.0f), meterRadius(0.05),
			 roi({-0.4f, -0.6f, -0.020f}, {0.4f, 0.6f, 0.4f}, "table_surface")
		{
		}
	};

	cv::Mat mask;
	std::map<std::pair<ros::Time, ros::Time>, Flow3D> flows3; // all R^3 vectors
	std::map<std::pair<ros::Time, ros::Time>, Flow2D> flows2;

	using LabelT = uint16_t;
	std::map<LabelT, std::vector<cv::Mat>> labelToTrackingLookup;
	std::map<LabelT, std::vector<std::vector<std::vector<bool>>>> labelToMasksLookup;

	TrackingOptions track_options;

	ros::Publisher vizPub;

	explicit
	Tracker(TrackingOptions _track_options = TrackingOptions());

	virtual
	~Tracker() = default;

	cv::Mat& getMask(const SensorHistoryBuffer& buffer);

	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup;

	virtual
	void track(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const std::map<ros::Time, cv::Mat>& masks = std::map<ros::Time, cv::Mat>());

//	virtual
//	void siftOnMask(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, LabelT label);

	virtual
	void reset();

};

/// \brief Estimates the rigid transformation
/// \param flow Points 'a' match to 'b'
/// \param bTa Transform that carries a to b
/// \return successful
bool estimateRigidTransform(const Tracker::Flow3D& flow, Eigen::Isometry3d& bTa);

}

#endif // MPS_VOXELS_TRACKER_H
