//
// Created by arprice on 12/11/18.
//

#ifndef MPS_SCENE_H
#define MPS_SCENE_H

#include "mps_voxels/PointT.h"
#include "mps_voxels/Indexes.h"
#include "mps_voxels/Object.h"
#include "mps_voxels/OccupancyData.h"
#include "mps_voxels/Manipulator.h"
#include "mps_voxels/MotionModel.h"
#include "segmentation_utils.h"
#include "mps_voxels/util/vector_less_than.hpp"

#include <octomap/OcTree.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <mps_msgs/AABBox2d.h>

#include <Eigen/StdVector>

#include <random>

namespace mps
{

class LocalOctreeServer;
class VoxelCompleter;
class RGBDSegmenter;

enum class FEATURE_AVAILABILITY
{
	REQUIRED,
	OPTIONAL,
	FORBIDDEN,
};

class Scenario
{
public:
	using Pose = moveit::Pose;
	// Lifetime properties

	tf::TransformListener* listener;
	tf::TransformBroadcaster* broadcaster;

	std::shared_ptr<LocalOctreeServer> mapServer;
	std::shared_ptr<VoxelCompleter> completionClient;
	std::shared_ptr<RGBDSegmenter> segmentationClient;

	robot_model::RobotModelPtr robotModel;
	std::vector<std::shared_ptr<Manipulator>> manipulators;
	std::map<std::string, std::shared_ptr<Manipulator>> jointToManipulator;
	robot_state::RobotStateConstPtr homeState;

	std::vector<std::pair<std::shared_ptr<shapes::Shape>, Pose>> staticObstacles;

	// Maximum extents of "tabletop"
	Eigen::Vector4f minExtent, maxExtent;

	mutable
	std::default_random_engine rng; // TODO: Should this be mutex-protected?

	std::map<std::string, bool> visualize;
	inline
	bool shouldVisualize(const std::string& channel) const
	{
		const auto& iter = visualize.find(channel);
		return (iter != visualize.end() && iter->second);
	}

	bool loadManipulators(robot_model::RobotModelPtr& pModel);
};

class Scene
{
public:
	using Pose = moveit::Pose;

	// Per-instant properties

	std::shared_ptr<const Scenario> scenario;
	std::map<std::string, std::shared_ptr<MotionModel>> selfModels;

	cv_bridge::CvImagePtr cv_rgb_ptr;
	cv_bridge::CvImagePtr cv_depth_ptr;
	image_geometry::PinholeCameraModel cameraModel;

	Pose worldTrobot; // transformation from robot to world
	Pose worldTcamera; // transformation from camera to world
	std::string worldFrame;
	std::string cameraFrame;

	// Maximum extent of active region of interest on tabletop
	Eigen::Vector4f minExtent, maxExtent;

	pcl::PointCloud<PointT>::Ptr cloud;
	pcl::PointCloud<PointT>::Ptr cropped_cloud;
	pcl::PointCloud<PointT>::Ptr pile_cloud;

	cv::Rect roi;
	cv_bridge::CvImage cv_rgb_cropped;
	cv_bridge::CvImage cv_depth_cropped;
	sensor_msgs::CameraInfo cam_msg_cropped;

	octomap::OcTree* sceneOctree;
	octomap::point3d_collection occludedPts;
	std::shared_ptr<octomap::OcTree> occlusionTree;

	// Properties that will depend on belief particle
	std::shared_ptr<SegmentationInfo> segInfo; ///< "Optimal" (best-guess) segmentation
	std::shared_ptr<OccupancyData> bestGuess;
//	std::map<ObjectIndex, pcl::PointCloud<PointT>::Ptr> segments;
//	std::map<uint16_t, ObjectIndex> labelToIndexLookup; ///< Carries body segmentation to object index in this scene
//	std::map<uint16_t, std::vector<xycoor>> labelToMaskLookup;
//	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup;

//	std::map<ObjectIndex, std::unique_ptr<Object>> objects;
//	std::map<ObjectIndex, octomap::point3d_collection> objectToShadow;
//	std::map<ObjectIndex, std::shared_ptr<octomap::OcTree>> completedSegments;
//	std::map<ObjectIndex, std::shared_ptr<shapes::Mesh>> approximateSegments;
//	std::map<octomap::point3d, ObjectIndex, vector_less_than<3, octomap::point3d>> coordToObject;
//	std::map<octomap::point3d, ObjectIndex, vector_less_than<3, octomap::point3d>> surfaceCoordToObject;
//	std::map<ObjectIndex, int> occludedBySegmentCount;
//
//	ObstructionList obstructions; // std::set<ObjectIndex>
//	std::shared_ptr<ObjectIndex> targetObjectID;

	ros::Time getTime() const { return cv_rgb_ptr->header.stamp; }

	bool convertImages(const sensor_msgs::ImageConstPtr& rgb_msg,
	                   const sensor_msgs::ImageConstPtr& depth_msg,
	                   const sensor_msgs::CameraInfo& cam_msg);

	enum { NeedsToAlign = (sizeof(Pose)%16)==0 };
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

class SceneProcessor
{
public:
	using Pose = Scenario::Pose;

	const std::shared_ptr<Scenario> scenario;
	const bool useMemory;
	const FEATURE_AVAILABILITY useShapeCompletion;

	explicit
	SceneProcessor(std::shared_ptr<Scenario> _scenario, bool memory = true, FEATURE_AVAILABILITY completion = FEATURE_AVAILABILITY::OPTIONAL)
	: scenario(std::move(_scenario)), useMemory(memory), useShapeCompletion(completion) {}

	bool loadAndFilterScene(Scene& s);

	bool callSegmentation(Scene& s);

	bool computeOcclusions(Scene& s);

	bool performSegmentation(const Scene& s, const std::shared_ptr<SegmentationInfo>& segHypo, OccupancyData& occupancy);

	bool buildObjects(const Scene& s, OccupancyData& occupancy);

	bool removeAccountedForOcclusion(octomap::point3d_collection& occludedPts,
	                                 std::shared_ptr<octomap::OcTree>& occlusionTree,
	                                 const OccupancyData& occupancy);

};

}
#endif // MPS_SCENE_H
