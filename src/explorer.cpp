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

#include <visualization_msgs/MarkerArray.h>
void publishOctree(std::shared_ptr<octomap::OcTree>& tree, const std::string& globalFrame)
{
	bool publishMarkerArray = (octreePub.getNumSubscribers()>0);

	visualization_msgs::MarkerArray occupiedNodesVis;
	occupiedNodesVis.markers.resize(tree->getTreeDepth()+1);

	if (publishMarkerArray)
	{

		for (octomap::OcTree::iterator it = tree->begin(tree->getTreeDepth()),
			     end = tree->end(); it != end; ++it)
		{
			if (tree->isNodeOccupied(*it))
			{
				unsigned idx = it.getDepth();

				geometry_msgs::Point cubeCenter;
				cubeCenter.x = it.getX();
				cubeCenter.y = it.getY();
				cubeCenter.z = it.getZ();

				occupiedNodesVis.markers[idx].points.push_back(cubeCenter);
			}
		}

		for (unsigned i = 0; i<occupiedNodesVis.markers.size(); ++i)
		{
			double size = tree->getNodeSize(i);

			occupiedNodesVis.markers[i].header.frame_id = globalFrame;
			occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
			occupiedNodesVis.markers[i].ns = "occlusion";
			occupiedNodesVis.markers[i].id = i;
			occupiedNodesVis.markers[i].type = visualization_msgs::Marker::CUBE_LIST;
			occupiedNodesVis.markers[i].scale.x = size;
			occupiedNodesVis.markers[i].scale.y = size;
			occupiedNodesVis.markers[i].scale.z = size;
//			if (!m_useColoredMap)
			occupiedNodesVis.markers[i].color.r = 0;
			occupiedNodesVis.markers[i].color.g = 0.2;
			occupiedNodesVis.markers[i].color.b = 1;
			occupiedNodesVis.markers[i].color.a = 1;

			if (occupiedNodesVis.markers[i].points.size()>0)
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
			else
				occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
		}

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
	listener->lookupTransform(info_msg->header.frame_id, globalFrame, ros::Time(0), cameraFrameInTableCoordinates);
	Eigen::Affine3d cameraTtable;
	tf::transformTFToEigen(cameraFrameInTableCoordinates, cameraTtable);

//	std::vector<Eigen::Vector3f>
	octomap::point3d_collection occluded_pts;
	std::shared_ptr<octomap::OcTree> occlusionTree(octree->create());

	const float resolution = static_cast<float>(octree->getResolution());
	const unsigned int d = octree->getTreeDepth();
//#pragma omp parallel for
	for (float ix = minExtent.x(); ix <= maxExtent.x(); ix += resolution)
	{
		for (float iy = minExtent.y(); iy <= maxExtent.y(); iy += resolution)
		{
			for (float iz = minExtent.z(); iz <= maxExtent.z(); iz += resolution)
			{
				Eigen::Vector3d p = cameraTtable * Eigen::Vector3d(ix, iy, iz);
				cv::Point3d worldPt(p.x(), p.y(), p.z());
				cv::Point2d imgPt = cameraModel.project3dToPixel(worldPt);
				const int buffer = 0;
				if (imgPt.x > buffer && imgPt.x < info_msg->width - buffer
				    && imgPt.y > buffer && imgPt.y < info_msg->height - buffer)
				{
//					#pragma omp critical
					if (!octree->search(ix, iy, iz, d))
					{
						//This cell is unknown
						octomap::point3d coord = octree->keyToCoord(octree->coordToKey(ix, iy, iz, d), d);
						occluded_pts.push_back(coord);
						occlusionTree->setNodeValue(coord, 0, false);
					}
				}
			}
		}
	}

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