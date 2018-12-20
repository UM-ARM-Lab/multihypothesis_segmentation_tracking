//
// Created by arprice on 12/18/18.
//

#ifndef PROJECT_MEMORYOBJECT_H
#define PROJECT_MEMORYOBJECT_H

#include "mps_voxels/MotionModel.h"
#include "mps_voxels/PointT.h"

#include <octomap/OcTree.h>

#include <Eigen/Geometry>
#include <memory>

class OctreeMotionModel : public RigidMotionModel
{
public:
	using Pose = MotionModel::Pose;
	void updateMembershipStructures() override;
	double membershipLikelihood(const Eigen::Vector3d& pt_global) const override;
	double membershipLikelihood(const Eigen::Vector3d& pt, const SensorModel& sensor) const override;

	bool observe(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& globalTcamera);
	bool observe(const pcl::PointCloud<PointT>::Ptr& cloud, const Pose& worldTcamera);

	std::shared_ptr<octomap::OcTree> octree;
};

#endif // PROJECT_MEMORYOBJECT_H
