//
// Created by arprice on 12/18/18.
//

#ifndef MPS_MEMORYOBJECT_H
#define MPS_MEMORYOBJECT_H

#include "mps_voxels/MotionModel.h"
#include "mps_voxels/PointT.h"

#include <octomap/OcTree.h>

#include <Eigen/Geometry>
#include <memory>

namespace mps
{

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

}

#endif // MPS_MEMORYOBJECT_H
