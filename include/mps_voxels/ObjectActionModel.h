//
// Created by kunhuang on 1/24/20.
//

#ifndef ARMLAB_WS_OBJECTACTIONMODEL_H
#define ARMLAB_WS_OBJECTACTIONMODEL_H

#include "mps_voxels/Tracker.h"
#include "mps_voxels/SiamTracker.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/util/assert.h"
#include "mps_voxels/PointT.h"
#include "mps_voxels/ROI.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/VoxelRegion.h"
#include "mps_voxels/Particle.h"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <octomap/OcTree.h>
#include <mps_msgs/ClusterRigidMotionsAction.h>

namespace mps
{
struct RigidTF
{
	Eigen::Vector3d linear;
	Eigen::Vector3d angular;
	int numInliers;
};

struct DecomposedRigidTF
{
	Eigen::Vector3d linear;
	Eigen::Vector3d e;
	double theta;
};

class ObjectActionModel
{
public:
	explicit ObjectActionModel(int n=1);

	std::vector<RigidTF> possibleRigidTFs; // clustered by Jlinkage

	int numSamples;
	std::vector<RigidTF> actionSamples;

	Eigen::Vector3d
	sampleActionFromMask(const cv::Mat& mask1, const cv::Mat& depth1,
	                     const cv::Mat& mask2, const cv::Mat& depth2,
	                     const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera);
	Eigen::Vector3d
	sampleActionFromMask(const std::vector<std::vector<bool>>& mask1, const cv::Mat& depth1,
	                     const std::vector<std::vector<bool>>& mask2, const cv::Mat& depth2,
	                     const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera);

	RigidTF icpManifoldSampler(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const std::map<ros::Time, cv::Mat>& masks, const moveit::Pose& worldTcamera);

	actionlib::SimpleActionClient<mps_msgs::ClusterRigidMotionsAction> jlinkageActionClient;

	bool clusterRigidBodyTransformation(const std::map<std::pair<ros::Time, ros::Time>, Tracker::Flow3D>& flows3camera, const moveit::Pose& worldTcamera);

	void sampleAction(SensorHistoryBuffer& buffer_out, SegmentationInfo& seg_out, std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, uint16_t label, mps_msgs::AABBox2d& bbox);

	void sampleAction(SensorHistoryBuffer& buffer_out, cv::Mat& firstFrameSeg, std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, uint16_t label, mps_msgs::AABBox2d& bbox);

	void weightedSampleSIFT(int n = 1);
};


std::shared_ptr<octomap::OcTree>
moveOcTree(const octomap::OcTree* octree, const RigidTF& action);

Particle
moveParticle(const Particle& inputParticle, const std::map<int, RigidTF>& labelToMotionLookup); // labelToMotionLookup should include all unique object labels in the inputParticle

}


#endif //ARMLAB_WS_OBJECTACTIONMODEL_H
