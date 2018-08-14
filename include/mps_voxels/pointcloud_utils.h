//
// Created by arprice on 7/31/18.
//

#ifndef MPS_VOXELS_POINTCLOUD_UTILS_H
#define MPS_VOXELS_POINTCLOUD_UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZRGB;

pcl::PointCloud<PointT>::Ptr filterInCameraFrame(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const double zMin = 0.1,
	const double zMax = 8.0,
	const int k = 15);

pcl::PointCloud<PointT>::Ptr cropInCameraFrame(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const Eigen::Vector4f& minExtent,
	const Eigen::Vector4f& maxExtent,
	const Eigen::Affine3d& worldTcamera);

std::vector<pcl::PointCloud<PointT>::Ptr> segment(
	pcl::PointCloud<PointT>::Ptr& cloud);

void getAABB(const pcl::PointCloud<PointT>& members, Eigen::Vector3f& min, Eigen::Vector3f& max);

void getBoundingCube(const pcl::PointCloud<PointT>& members, Eigen::Vector3f& min, Eigen::Vector3f& max);

#endif // MPS_VOXELS_POINTCLOUD_UTILS_H
