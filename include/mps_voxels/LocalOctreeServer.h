//
// Created by arprice on 9/10/18.
//

#ifndef PROJECT_LOCALOCTREESERVER_H
#define PROJECT_LOCALOCTREESERVER_H

#include "mps_voxels/PointT.h"

#include <octomap/OcTree.h>
#include <ros/node_handle.h>


template <typename T>
void setIfMissing(ros::NodeHandle& nh, const std::string& param_name, const T& param_val)
{
	if (!nh.hasParam(param_name))
	{
		nh.setParam(param_name, param_val);
	}
}

template <typename Point>
void setBBox(const Point& min, const Point& max, octomap::OcTree* ot)
{
	octomap::point3d om_min{(float)min.x(), (float)min.y(), (float)min.z()};
	octomap::point3d om_max{(float)max.x(), (float)max.y(), (float)max.z()};
	ot->setBBXMin(om_min);
	ot->setBBXMax(om_max);
	ot->useBBXLimit(true);
}

/**
 * @brief Takes a point cloud in camera coordinates (z=depth), a point of view, and incorporates it into a tree map
 * @param cloud
 * @param worldTcamera
 * @param tree
 */
void insertCloudInOctree(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& worldTcamera, octomap::OcTree* tree);

class LocalOctreeServer// : public octomap_server::OctomapServer
{
public:
	using OcTreeT = octomap::OcTree;

	double m_res;
	std::string m_worldFrameId; // the map frame
	std::unique_ptr<OcTreeT> m_octree;

	LocalOctreeServer(const ros::NodeHandle& private_nh_ = ros::NodeHandle("~"));

	void insertCloud(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& worldTcamera);

	const std::string& getWorldFrame() const { return this->m_worldFrameId; }
	const OcTreeT* getOctree() const { return this->m_octree.get(); }
	OcTreeT* getOctree() { return this->m_octree.get(); }
};

#endif // PROJECT_LOCALOCTREESERVER_H
