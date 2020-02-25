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
	moveit::Pose tf;
	int numInliers = -1;
};
//struct RigidTF
//{
//	Eigen::Vector3d linear;
//	Eigen::Vector3d angular;
//	int numInliers = -1;
//};

struct DecomposedRigidTF
{
	Eigen::Vector3d linear;
	Eigen::Vector3d e;
	double theta;
};

class Scenario;

class ObjectActionModel
{
public:
	using TimePoseLookup = std::map<std::pair<ros::Time, ros::Time>, moveit::Pose, std::less<>, Eigen::aligned_allocator<moveit::Pose>>;

	explicit ObjectActionModel(std::shared_ptr<const Scenario> scenario_, int n=1);

	std::shared_ptr<const Scenario> scenario;

	std::vector<RigidTF> siftRigidTFs; // clustered by Jlinkage
	RigidTF icpRigidTF;

	int numSamples;
	std::vector<RigidTF> actionSamples;

	Eigen::Vector3d
	sampleActionFromMask(const cv::Mat& mask1, const cv::Mat& depth1,
	                     const cv::Mat& mask2, const cv::Mat& depth2,
	                     const image_geometry::PinholeCameraModel& cameraModel, const mps::Pose& worldTcamera);

	RigidTF icpManifoldSampler(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const std::map<ros::Time, cv::Mat>& masks, const mps::Pose& worldTcamera);

	TimePoseLookup
	icpManifoldSequencialSampler(const std::vector<ros::Time>& steps, const SensorHistoryBuffer& buffer, const std::map<ros::Time, cv::Mat>& masks, const mps::Pose& worldTcamera);

	actionlib::SimpleActionClient<mps_msgs::ClusterRigidMotionsAction> jlinkageActionClient;

	bool clusterRigidBodyTransformation(const std::map<std::pair<ros::Time, ros::Time>, Tracker::Flow3D>& flows3camera, const mps::Pose& worldTcamera);

//	bool sampleAction(const SensorHistoryBuffer& buffer_out, SegmentationInfo& seg_out, std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, uint16_t label, mps_msgs::AABBox2d& bbox);

	bool sampleAction(const SensorHistoryBuffer& buffer_out, const cv::Mat& firstFrameSeg, const cv::Rect& roi, std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker, uint16_t label, mps_msgs::AABBox2d& bbox);

	void weightedSampleSIFT(int n = 1);

	bool isSiamMaskValidICPbased(const pcl::PointCloud<PointT>::Ptr& initCloudSegment, const pcl::PointCloud<PointT>::Ptr& lastCloudSegment,
	                             const mps::Pose& worldTcamera, const double& scoreThreshold,
	                             const bool& useGuess = false, const Eigen::Matrix4f& guessCamera = Eigen::Matrix4f::Identity());
};

Eigen::Matrix4d convertTFformat(Eigen::Vector3d linear, Eigen::Vector3d angular);

std::shared_ptr<octomap::OcTree>
moveOcTree(const octomap::OcTree* octree, const RigidTF& action);

Particle
moveParticle(const Particle& inputParticle, const std::map<int, RigidTF>& labelToMotionLookup); // labelToMotionLookup should include all unique object labels in the inputParticle

}


#endif //ARMLAB_WS_OBJECTACTIONMODEL_H
