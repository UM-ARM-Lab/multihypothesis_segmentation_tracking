
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/pointcloud_utils.h"
#include <mps_voxels/CompleteShape.h>
//#include <octomap/octomap.h>
//#include <octomap_msgs/conversions.h>
//#include <octomap_msgs/GetOctomap.h>
#include <Eigen/Geometry>

#include <std_msgs/ByteMultiArray.h>

#include <ros/ros.h>
#include <memory>
#include <cassert>

namespace om = octomap;

std::shared_ptr<OctreeRetriever> mapClient;
std::shared_ptr<VoxelCompleter> completionClient;
ros::Publisher octreePub;

void publishOctree(std::shared_ptr<octomap::OcTree>& tree, const std::string& globalFrame)
{
	bool publishMarkerArray = (octreePub.getNumSubscribers()>0);

	if (publishMarkerArray)
	{
		visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(tree, globalFrame);

		octreePub.publish(occupiedNodesVis);
	}
}


// Point cloud utilities
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf_conversions/tf_eigen.h>
//#include <abstract.h>

using PointT = pcl::PointXYZRGB;

std::shared_ptr<tf::TransformListener> listener;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
	// Get Octree
	std::shared_ptr<om::OcTree> octree;
	std::string globalFrame;
	std::tie(octree, globalFrame) = mapClient->getOctree();
	if (!octree)
	{
		ROS_ERROR("Octree lookup failed.");
		return;
	}

	// Transform point cloud to local frame
	if (!listener->waitForTransform(globalFrame, cloudMsg->header.frame_id, ros::Time(0), ros::Duration(5.0)))
	{
		ROS_WARN_STREAM("Failed to look up transform between '" << globalFrame << "' and '" << cloudMsg->header.frame_id << "'.");
		return;
	}
//	sensor_msgs::PointCloud2 transformedCloud;
//	pcl_ros::transformPointCloud(globalFrame, *cloudMsg, transformedCloud, *listener);

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
//	pcl::fromROSMsg(transformedCloud, *cloud);
	pcl::fromROSMsg(*cloudMsg, *cloud);

	// Filter box
	// Coordinates in table ("global") frame
	Eigen::Vector4f maxExtent(0.5, 1, 1, 1);
	Eigen::Vector4f minExtent(-0.5, -1, -0.25, 1);

	tf::StampedTransform tableFrameInCameraCoordinates;
	listener->lookupTransform(globalFrame, cloudMsg->header.frame_id, ros::Time(0), tableFrameInCameraCoordinates);
	Eigen::Affine3d worldTcamera;
	tf::transformTFToEigen(tableFrameInCameraCoordinates, worldTcamera);

	pcl::PointCloud<PointT>::Ptr cropped_cloud = crop(cloud, minExtent, maxExtent, worldTcamera);
	if (cropped_cloud->empty())
	{
		ROS_WARN("Filtered cloud contains no points.");
		return;
	}

	// Perform segmentation
	std::vector<pcl::PointCloud<PointT>::Ptr> segments = segment(cropped_cloud);

	std::vector<std::shared_ptr<om::OcTree>> completedClusters;
	for (const pcl::PointCloud<PointT>::Ptr& cluster_cloud : segments)
	{
		// Compute bounding box
		Eigen::Vector3f min, max;
		getBoundingCube(*cluster_cloud, min, max);

		std::shared_ptr<om::OcTree> subtree(octree->create());
		for (const PointT& pt : *cluster_cloud)
		{
			Eigen::Vector3f worldPt = worldTcamera.cast<float>() * pt.getVector3fMap();
			assert(worldPt.cwiseMax(maxExtent.head<3>()) == maxExtent.head<3>());
			assert(worldPt.cwiseMin(minExtent.head<3>()) == minExtent.head<3>());

			om::OcTreeNode* node = subtree->setNodeValue(worldPt.x(), worldPt.y(), worldPt.z(), std::numeric_limits<float>::infinity(), false);
//			assert(node);
//			assert(node->getOccupancy() > 0.5);
//			assert(octree->search(worldPt.x(), worldPt.y(), worldPt.z())->getOccupancy() > 0.5); // May fail if server is behind
//			assert(subtree->search(worldPt.x(), worldPt.y(), worldPt.z())->getOccupancy() > 0.5);
		}
		completionClient->completeShape(min, max, worldTcamera.cast<float>(), octree, subtree);

		completedClusters.push_back(subtree);

		publishOctree(subtree, globalFrame);
	}


}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "shape_completion_manager");
	ros::NodeHandle nh;

	listener = std::make_shared<tf::TransformListener>();

//	localMapPub = nh.advertise<std_msgs::ByteMultiArray>("local_occupancy", 1, true);
	octreePub = nh.advertise<visualization_msgs::MarkerArray>("predicted_points", 1, true);
	mapClient = std::make_shared<OctreeRetriever>(nh);
	completionClient = std::make_shared<VoxelCompleter>(nh);

	ros::Subscriber sub = nh.subscribe ("kinect2_victor_head/qhd/points", 1, cloud_cb);

//	getOccupancy(om::point3d(-0.1f, -0.1f, -0.1f), om::point3d(0.1f, 0.1f, 0.1f));

//	ros::spinOnce();
	ros::spin();

	return 0;
}