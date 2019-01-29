//
// Created by arprice on 12/11/18.
//

#ifndef MPS_SCENE_H
#define MPS_SCENE_H

#include "mps_voxels/PointT.h"
#include "mps_voxels/ObjectIndex.h"
#include "mps_voxels/Manipulator.h"
#include "mps_voxels/MotionModel.h"
#include "segmentation_utils.h"
#include "mps_voxels/vector_less_than.h"

#include <moveit/collision_detection/world.h>

#include <octomap/OcTree.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/StdVector>

class LocalOctreeServer;
class VoxelCompleter;
class RGBDSegmenter;

enum class FEATURE_AVAILABILITY
{
	REQUIRED,
	OPTIONAL,
	FORBIDDEN,
};

class Object
{
public:
	const ObjectIndex index;
	std::shared_ptr<octomap::OcTree> occupancy;
	std::shared_ptr<shapes::Mesh> approximation;
	octomap::point3d_collection points;
	octomap::point3d_collection shadow;

	Object(const ObjectIndex i, const std::shared_ptr<octomap::OcTree>& tree);
};

class Scenario
{
public:
	using Pose = Eigen::Affine3d;
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

	std::default_random_engine rng;

	bool loadManipulators(robot_model::RobotModelPtr& pModel);
};

class Scene
{
public:
	using Pose = Scenario::Pose;
	using ObstructionList = std::map<ObjectIndex, double>;

	// Per-instant properties

	bool visualize = true;

	std::shared_ptr<Scenario> scenario;
	std::map<std::string, std::shared_ptr<MotionModel>> selfModels;

	cv_bridge::CvImagePtr cv_rgb_ptr;
	cv_bridge::CvImagePtr cv_depth_ptr;
	image_geometry::PinholeCameraModel cameraModel;

	Pose worldTrobot;
	Pose worldTcamera;
	std::string worldFrame;
	std::string cameraFrame;
	Eigen::Vector4f minExtent, maxExtent;

	pcl::PointCloud<PointT>::Ptr cloud;
	pcl::PointCloud<PointT>::Ptr cropped_cloud;
	pcl::PointCloud<PointT>::Ptr pile_cloud;

	cv::Rect roi;
	cv_bridge::CvImage cv_rgb_cropped;
	cv_bridge::CvImage cv_depth_cropped;
	sensor_msgs::CameraInfo cam_msg_cropped;
	std::shared_ptr<SegmentationInfo> segInfo;
	std::map<ObjectIndex, pcl::PointCloud<PointT>::Ptr> segments;
	std::map<uint16_t, ObjectIndex> labelToIndexLookup; ///< Carries body segmentation to object index in this scene

	octomap::OcTree* sceneOctree;
	std::map<ObjectIndex, std::unique_ptr<Object>> objects;
//	std::map<ObjectIndex, octomap::point3d_collection> objectToShadow;
//	std::map<ObjectIndex, std::shared_ptr<octomap::OcTree>> completedSegments;
//	std::map<ObjectIndex, std::shared_ptr<shapes::Mesh>> approximateSegments;
	std::map<octomap::point3d, ObjectIndex, vector_less_than<3, octomap::point3d>> coordToObject;
	std::map<octomap::point3d, ObjectIndex, vector_less_than<3, octomap::point3d>> surfaceCoordToObject;
	octomap::point3d_collection occludedPts;
	std::shared_ptr<octomap::OcTree> occlusionTree;
	std::map<ObjectIndex, int> occludedBySegmentCount;

	ObstructionList obstructions; // std::set<ObjectIndex>
	std::shared_ptr<ObjectIndex> targetObjectID;

	static const std::string CLUTTER_NAME;
	collision_detection::WorldConstPtr collisionWorld;
	collision_detection::WorldPtr computeCollisionWorld();

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

	bool performSegmentation(Scene& s);

	bool buildObjects(Scene& s);

};

#endif // MPS_SCENE_H
