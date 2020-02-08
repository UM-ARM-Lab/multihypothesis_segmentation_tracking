//
// Created by kunhuang on 1/24/20.
//

#ifndef ARMLAB_WS_OBJECTACTIONMODEL_H
#define ARMLAB_WS_OBJECTACTIONMODEL_H

#include "mps_voxels/Tracker.h"
#include "mps_voxels/SiamTracker.h"
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
struct rigidTF
{
	Eigen::Vector3d linear;
	Eigen::Vector3d angular;
	int numInliers;
};

class objectActionModel
{
public:
	explicit objectActionModel(int n=1);

	std::vector<rigidTF> possibleRigidTFs;

	int numSamples;
	std::vector<rigidTF> actionSamples;

	Eigen::Vector3d
	sampleActionFromMask(const cv::Mat& mask1, const cv::Mat& depth1,
	                     const cv::Mat& mask2, const cv::Mat& depth2,
	                     const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera);
	Eigen::Vector3d
	sampleActionFromMask(const std::vector<std::vector<bool>>& mask1, const cv::Mat& depth1,
	                     const std::vector<std::vector<bool>>& mask2, const cv::Mat& depth2,
	                     const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera);

	actionlib::SimpleActionClient<mps_msgs::ClusterRigidMotionsAction> jlinkageActionClient;

	bool clusterRigidBodyTransformation(const std::map<std::pair<ros::Time, ros::Time>, Tracker::Flow3D>& flows3camera, const moveit::Pose& worldTcamera);

	void sampleAction(SensorHistoryBuffer& buffer_out, SegmentationInfo& seg_out, std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, uint16_t label, mps_msgs::AABBox2d& bbox);

	void weightedSampleSIFT(int n = 1);
};


mps::VoxelSegmentation
moveParticle(const mps::VoxelSegmentation& particle, const Eigen::Vector3d& action);

std::shared_ptr<octomap::OcTree>
moveOcTree(const octomap::OcTree* octree, const rigidTF& action);
}


#endif //ARMLAB_WS_OBJECTACTIONMODEL_H
