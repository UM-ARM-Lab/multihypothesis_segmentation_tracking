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

#ifndef MPS_OCCUPANCYDATA_H
#define MPS_OCCUPANCYDATA_H

#include "mps_voxels/Indexes.h"
#include "mps_voxels/VoxelRegion.h"
#include "mps_voxels/PointT.h"
#include "mps_voxels/Object.h"
#include "mps_voxels/SegmentationInfo.h"

#include "mps_voxels/util/vector_less_than.hpp"
#include "mps_voxels/moveit_pose_type.h"

//#include <moveit/collision_detection/world.h>
//#include <moveit/macros/class_forward.h>

#include <boost/bimap.hpp>

//namespace collision_detection
//{
//MOVEIT_CLASS_FORWARD(World)
//}

namespace image_geometry
{
class PinholeCameraModel;
}

namespace mps
{

class Scene;

struct OccupancyData
{
	// Constructor to initialize valid region, empty state
	explicit
	OccupancyData(std::shared_ptr<VoxelRegion> region);

	OccupancyData(std::shared_ptr<VoxelRegion> region, VoxelRegion::VertexLabels vertexState);

	// Domain properties shared by this particle
	std::shared_ptr<VoxelRegion> voxelRegion;

	// This is the canonical representation of the segmentation state
	VoxelRegion::VertexLabels vertexState;

	// This is the dual form
	VoxelRegion::EdgeState edgeState;

	// Cached set of object labels
	std::set<ObjectIndex> uniqueObjectLabels;

	std::map<ObjectIndex, pcl::PointCloud<PointT>::Ptr> segments;
	boost::bimap<uint16_t, ObjectIndex> labelToIndexLookup; ///< Carries body segmentation to object index in this scene

	// Objects in this scene
	std::map<ObjectIndex, std::shared_ptr<Object>> objects;

	ObjectIndex coordToObject(const Eigen::Vector3d& pt) const;
	std::map<ObjectIndex, int> occludedBySegmentCount;

	using ObstructionList = std::map<ObjectIndex, double>;
	ObstructionList obstructions; // std::set<ObjectIndex>
	std::shared_ptr<ObjectIndex> targetObjectID;
};

cv::Rect2d occupancyToROI(const OccupancyData& state, const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera);
cv::Rect2d workspaceToROI(const VoxelRegion& region, const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera);
cv::Mat rayCastOccupancy(const OccupancyData& occupancy, const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera, const cv::Rect& roi, const int& step = 1);

}

#endif // MPS_OCCUPANCYDATA_H
