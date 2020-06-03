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

#ifndef MPS_VOXELS_POINTCLOUD_UTILS_H
#define MPS_VOXELS_POINTCLOUD_UTILS_H

#include "mps_voxels/PointT.h"

namespace mps
{

pcl::PointCloud<PointT>::Ptr filterInCameraFrame(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const float zMin = 0.1,
	const float zMax = 8.0,
	const int k = 15,
	const float stdDev = 1.0);

pcl::PointCloud<PointT>::Ptr cropInCameraFrame(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const Eigen::Vector4f& minExtent,
	const Eigen::Vector4f& maxExtent,
	const Eigen::Affine3d& worldTcamera);

pcl::PointCloud<PointT>::Ptr filterOutliers(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const int k = 15,
	const double stddev = 1.0);

pcl::PointCloud<PointT>::Ptr filterPlane(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const double distanceThreshold = 0.01,
	const Eigen::Vector3f& normal = Eigen::Vector3f::Zero());

pcl::PointCloud<PointT>::Ptr filterSmallClusters(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const int clusterThreshold = 100,
	const float clusterDistance = 0.01);

std::vector<pcl::PointCloud<PointT>::Ptr> segment(
	pcl::PointCloud<PointT>::Ptr& cloud);

void getAABB(const pcl::PointCloud<PointT>& members, Eigen::Vector3d& min, Eigen::Vector3d& max);

void getBoundingCube(const pcl::PointCloud<PointT>& members, Eigen::Vector3d& min, Eigen::Vector3d& max);

}

#endif // MPS_VOXELS_POINTCLOUD_UTILS_H
