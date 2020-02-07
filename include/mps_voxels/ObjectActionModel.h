//
// Created by kunhuang on 1/24/20.
//

#ifndef ARMLAB_WS_OBJECTACTIONMODEL_H
#define ARMLAB_WS_OBJECTACTIONMODEL_H

#include "mps_voxels/Tracker.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/assert.h"
#include "mps_voxels/PointT.h"
#include "mps_voxels/ROI.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/VoxelSegmentation.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <octomap/OcTree.h>
#include <mps_msgs/ClusterRigidMotionsAction.h>

namespace mps
{

class objectActionModel
{
public:
	objectActionModel();

	Eigen::Vector3d
	sampleActionFromMask(const cv::Mat& mask1, const cv::Mat& depth1,
	                     const cv::Mat& mask2, const cv::Mat& depth2,
	                     const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera);
	Eigen::Vector3d
	sampleActionFromMask(const std::vector<std::vector<bool>>& mask1, const cv::Mat& depth1,
	                     const std::vector<std::vector<bool>>& mask2, const cv::Mat& depth2,
	                     const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera);

	actionlib::SimpleActionClient<mps_msgs::ClusterRigidMotionsAction> jlinkageActionClient;

	void clusterRigidBodyTransformation(const std::map<std::pair<ros::Time, ros::Time>, Tracker::Flow3D>& flows3camera, const moveit::Pose& worldTcamera);
};


mps::VoxelSegmentation
moveParticle(const mps::VoxelSegmentation& particle, const Eigen::Vector3d& action);

std::shared_ptr<octomap::OcTree>
moveOcTree(const octomap::OcTree* octree, const Eigen::Vector3d& action);
}


#endif //ARMLAB_WS_OBJECTACTIONMODEL_H
