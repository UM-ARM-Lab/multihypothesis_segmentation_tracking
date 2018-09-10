//
// Created by arprice on 9/10/18.
//

#include "mps_voxels/LocalOctreeServer.h"

#include <octomap/Pointcloud.h>
#include <pcl_ros/transforms.h>

void insertCloudInOctree(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& worldTcamera, octomap::OcTree* tree)
{
	if (cloud->empty()) { throw std::runtime_error("Trying to insert empty cloud!"); }

	pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>());
	pcl::transformPointCloud (*cloud, *transformed_cloud, worldTcamera);

	octomap::point3d min = tree->getBBXMin(), max = tree->getBBXMax();
	for (int i = 0; i < 3; ++i)
	{
		min(i) = std::min(min(i), (float)worldTcamera.translation()[i]);
		max(i) = std::max(max(i), (float)worldTcamera.translation()[i]);
	}
	setBBox(min, max, tree);

	octomap::Pointcloud pc;
	pc.reserve(transformed_cloud->size());
	for (const PointT& pt : *transformed_cloud)
	{
		pc.push_back(pt.x, pt.y, pt.z);
		assert(pt.x > min(0)); assert(pt.x < max(0));
		assert(pt.y > min(1)); assert(pt.y < max(1));
		assert(pt.z > min(2)); assert(pt.z < max(2));
	}
	assert(pc.size() == cloud->size());
	octomap::point3d origin((float)worldTcamera.translation().x(),
	                        (float)worldTcamera.translation().y(),
	                        (float)worldTcamera.translation().z());
//	m_octree->insertPointCloud(pc, origin, -1, true, false);
//	m_octree->insertPointCloudRays(pc, origin, -1, true);

	tree->insertPointCloud(pc, origin, -1, false, true);
	tree->updateInnerOccupancy();

	assert(tree->size() > 0);
}

LocalOctreeServer::LocalOctreeServer(const ros::NodeHandle& private_nh_)
	: m_res(0.05),
	  m_worldFrameId("/map"),
	  m_octree(nullptr)
{
	private_nh_.param("resolution", m_res, m_res);
	private_nh_.param("frame_id", m_worldFrameId, m_worldFrameId);

	m_octree = std::make_unique<OcTreeT>(m_res);


	Eigen::Vector3d min, max;
	m_octree->getMetricMin(min.x(), min.y(), min.z());
	m_octree->getMetricMax(max.x(), max.y(), max.z());
	setBBox(min, max, m_octree.get());
}

void LocalOctreeServer::insertCloud(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& worldTcamera)
{
	insertCloudInOctree(cloud, worldTcamera, m_octree.get());
}