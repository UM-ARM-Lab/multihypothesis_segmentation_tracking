//
// Created by arprice on 7/24/18.
//

// Octree utilities
#include "mps_voxels/octree_utils.h"
//#include <octomap/octomap.h>
//#include <octomap_msgs/conversions.h>
//#include <octomap_msgs/GetOctomap.h>

#include <Eigen/Geometry>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <ros/ros.h>

#include <algorithm>
#include <memory>
#include <random>

std::shared_ptr<OctreeRetriever> mapClient;
ros::Publisher octreePub;
ros::Publisher targetPub;
std::shared_ptr<tf::TransformListener> listener;

void publishOctree(std::shared_ptr<octomap::OcTree>& tree, const std::string& globalFrame)
{
	bool publishMarkerArray = (octreePub.getNumSubscribers()>0);

	if (publishMarkerArray)
	{
		visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(tree, globalFrame);

		octreePub.publish(occupiedNodesVis);
	}
}

void camera_cb(const sensor_msgs::CameraInfoConstPtr& info_msg)
{
	std::cerr << "Got camera info." << *info_msg << std::endl;
	image_geometry::PinholeCameraModel cameraModel;
	cameraModel.fromCameraInfo(info_msg);

	Eigen::Vector4f maxExtent(0.5, 1, 1, 1);
	Eigen::Vector4f minExtent(-0.5, -1, -0.25, 1);

	// Get Octree
	auto octreeInfo = mapClient->getOctree();
	std::shared_ptr<octomap::OcTree> octree = octreeInfo.first;
	std::string globalFrame = octreeInfo.second;
	if (!octree)
	{
		ROS_ERROR("Octree lookup failed.");
		return;
	}

	tf::StampedTransform cameraFrameInTableCoordinates;
	listener->lookupTransform(cameraModel.tfFrame(), globalFrame, ros::Time(0), cameraFrameInTableCoordinates);
	Eigen::Affine3d cameraTtable;
	tf::transformTFToEigen(cameraFrameInTableCoordinates, cameraTtable);

	octomap::point3d_collection occluded_pts;
	std::shared_ptr<octomap::OcTree> occlusionTree;
	std::tie(occluded_pts, occlusionTree) = getOcclusionsInFrame(octree, cameraModel, cameraTtable, minExtent.head<3>(), maxExtent.head<3>());

	std::random_device rd;
	std::mt19937 g(rd());

	std::shuffle(occluded_pts.begin(), occluded_pts.end(), g);
	geometry_msgs::Point pt;
	pt.x = occluded_pts.front().x();
	pt.y = occluded_pts.front().y();
	pt.z = occluded_pts.front().z();
	targetPub.publish(pt);

//	occlusionTree->
	publishOctree(occlusionTree, globalFrame);
	std::cerr << "Published " << occluded_pts.size() << " points." << std::endl;


	ros::shutdown();
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "entropy_estimator");
	ros::NodeHandle nh;

	listener = std::make_shared<tf::TransformListener>();

	octreePub = nh.advertise<visualization_msgs::MarkerArray>("occluded_points", 1, true);
	targetPub = nh.advertise<geometry_msgs::Point>("target_point", 1, false);
	mapClient = std::make_shared<OctreeRetriever>(nh);
	ros::Subscriber camInfoSub = nh.subscribe("kinect2_victor_head/qhd/camera_info", 1, camera_cb);

//	ros::Subscriber sub = nh.subscribe ("kinect2_roof/qhd/points", 1, cloud_cb);



	//(new octomap::OcTree(d));


	ros::spin();

	return 0;
}