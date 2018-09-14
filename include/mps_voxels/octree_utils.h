//
// Created by arprice on 7/24/18.
//

#ifndef MPS_VOXELS_OCTREE_UTILS_H
#define MPS_VOXELS_OCTREE_UTILS_H

#include <octomap/octomap.h>

#include <image_geometry/pinhole_camera_model.h>

#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>

#include <Eigen/Geometry>
#include <memory>


bool isSpeckleNode(const octomap::OcTreeKey& nKey, const octomap::OcTree* octree);

class OctreeRetriever
{
public:
	explicit
	OctreeRetriever(ros::NodeHandle& nh);
	ros::ServiceClient mapClient;
	std::pair<std::shared_ptr<octomap::OcTree>, std::string> getOctree();
};

class VoxelCompleter
{
public:
	explicit
	VoxelCompleter(ros::NodeHandle& nh);

	mutable
	ros::ServiceClient completionClient;

	// min and max specified in camera frame
	void completeShape(
		const Eigen::Vector3f& min,
		const Eigen::Vector3f& max,
		const Eigen::Affine3f& worldTcamera,
		octomap::OcTree* octree,
		octomap::OcTree* subtree,
		bool updateMainTree = true,
		const int resolution = 64) const;
};


octomap::point3d_collection getPoints(const octomap::OcTree* octree);

std::pair<octomap::point3d_collection, std::shared_ptr<octomap::OcTree>> getOcclusionsInFOV(
	const octomap::OcTree* octree,
	const image_geometry::PinholeCameraModel& cameraModel,
	const Eigen::Affine3d& cameraTworld,
	const Eigen::Vector3f& minExtent,
	const Eigen::Vector3f& maxExtent);

visualization_msgs::MarkerArray visualizeOctree(octomap::OcTree* tree, const std::string& globalFrame = "world", const std_msgs::ColorRGBA* base_color = nullptr);

#endif // MPS_VOXELS_OCTREE_UTILS_H
