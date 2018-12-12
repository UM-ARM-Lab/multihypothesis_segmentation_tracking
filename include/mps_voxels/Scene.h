//
// Created by arprice on 12/11/18.
//

#ifndef MPS_SCENE_H
#define MPS_SCENE_H

#include "mps_voxels/PointT.h"
#include "mps_voxels/Manipulator.h"
#include "mps_voxels/MotionModel.h"
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

class Scene
{
public:
	using Pose = Eigen::Affine3d;

	bool visualize = true;
	std::shared_ptr<tf::TransformListener> listener;
	std::shared_ptr<tf::TransformBroadcaster> broadcaster;

	robot_model::RobotModelPtr robotModel;
	std::vector<std::shared_ptr<Manipulator>> manipulators;
	std::map<std::string, std::shared_ptr<Manipulator>> jointToManipulator;

	std::vector<std::pair<std::shared_ptr<shapes::Shape>, Pose>> staticObstacles;
	std::map<std::string, std::shared_ptr<MotionModel>> motionModels;

	std::default_random_engine rng;

	Pose worldTrobot;
	Pose worldTcamera;
	std::string worldFrame;
	std::string cameraFrame;
	Eigen::Vector4f minExtent, maxExtent;
	image_geometry::PinholeCameraModel cameraModel;

	cv_bridge::CvImagePtr cv_rgb_ptr;
	cv_bridge::CvImagePtr cv_depth_ptr;
	cv_bridge::CvImagePtr cv_seg_ptr;

	pcl::PointCloud<PointT>::Ptr cloud;
	pcl::PointCloud<PointT>::Ptr cropped_cloud;
	pcl::PointCloud<PointT>::Ptr pile_cloud;

	cv::Rect roi;
	cv::Mat rgb_cropped;
	cv::Mat depth_cropped;
	std::vector<pcl::PointCloud<PointT>::Ptr> segments;
	std::map<uint16_t, int> labelToIndexLookup;

	octomap::OcTree* sceneOctree;
	std::map<int, octomap::point3d_collection> objectToShadow;
	std::map<octomap::point3d, int, vector_less_than<3, octomap::point3d>> coordToObject;
	std::map<octomap::point3d, int, vector_less_than<3, octomap::point3d>> surfaceCoordToObject;
	std::vector<std::shared_ptr<octomap::OcTree>> completedSegments;
	std::vector<std::shared_ptr<shapes::Mesh>> approximateSegments;
	octomap::point3d_collection occludedPts;
	std::shared_ptr<octomap::OcTree> occlusionTree;

	std::set<int> obstructions;

	static const std::string CLUTTER_NAME;
	collision_detection::WorldConstPtr collisionWorld;
	collision_detection::WorldPtr computeCollisionWorld();

	std::shared_ptr<LocalOctreeServer> mapServer;
	std::shared_ptr<VoxelCompleter> completionClient;
	std::shared_ptr<RGBDSegmenter> segmentationClient;


	enum { NeedsToAlign = (sizeof(Pose)%16)==0 };
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)

	bool loadManipulators(robot_model::RobotModelPtr& pModel);

	bool loadAndFilterScene(const sensor_msgs::ImageConstPtr& rgb_msg,
	                        const sensor_msgs::ImageConstPtr& depth_msg,
	                        const sensor_msgs::CameraInfoConstPtr& cam_msg);
};

#endif // MPS_SCENE_H
