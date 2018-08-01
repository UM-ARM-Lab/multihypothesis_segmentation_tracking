//
// Created by arprice on 7/31/18.
//

#ifndef PROJECT_POINTCLOUD_UTILS_H
#define PROJECT_POINTCLOUD_UTILS_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointT = pcl::PointXYZRGB;

pcl::PointCloud<PointT>::Ptr crop(pcl::PointCloud<PointT>::Ptr& cloud,
                                  const Eigen::Vector4f& minExtent,
                                  const Eigen::Vector4f& maxExtent,
                                  const Eigen::Affine3d& worldTcamera);

std::vector<pcl::PointCloud<PointT>::Ptr> segment(
	pcl::PointCloud<PointT>::Ptr& cloud);

template <typename PointT>
void getAABB(const pcl::PointCloud<PointT>& members, Eigen::Vector3f& min, Eigen::Vector3f& max)
{
	const int DIM = 3;
	for (int d=0; d < DIM; ++d)
	{
		min[d] = std::numeric_limits<float>::infinity();
		max[d] = -std::numeric_limits<float>::infinity();
	}

	for (const PointT& p : members)
	{
		for (int d=0; d < DIM; ++d)
		{
			min[d] = std::min(min[d], p.getVector3fMap()[d]);
			max[d] = std::max(max[d], p.getVector3fMap()[d]);
		}
	}
}

void getBoundingCube(const pcl::PointCloud<PointT>& members, Eigen::Vector3f& min, Eigen::Vector3f& max);

#endif //PROJECT_POINTCLOUD_UTILS_H
