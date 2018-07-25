
#include "mps_voxels/octree_utils.h"
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
ros::Publisher localMapPub;

// min and max now specified in camera frame
void getOccupancy(const Eigen::Vector3f& min, const Eigen::Vector3f& max, const Eigen::Affine3f& worldTcamera,
                  std::shared_ptr<om::OcTree>& octree, std::shared_ptr<om::OcTree>& subtree,
                  const int resolution = 64)
{
	assert(resolution > 0);
	assert(max.x() > min.x());
	assert(max.y() > min.y());
	assert(max.z() > min.z());

//	// Get Octree
//	auto octreeInfo = getOctree();
//	std::shared_ptr<om::OcTree> octree = octreeInfo.first;
//	std::string globalFrame = octreeInfo.second;
//	if (!octree)
//	{
//		ROS_ERROR("Octree lookup failed.");
//		return;
//	}

	std::cerr << octree->getNumLeafNodes() << ", " << octree->getResolution() << std::endl;
	if (subtree)
	{
		std::cerr << subtree->getNumLeafNodes() << ", " << subtree->getResolution() << std::endl;
	}

	std_msgs::ByteMultiArray arrayMsg;
	std_msgs::MultiArrayDimension dim;

	dim.label = "x"; dim.size = (unsigned)resolution; dim.stride = (unsigned)resolution*resolution*resolution;
	arrayMsg.layout.dim.push_back(dim);
	dim.label = "y"; dim.size = (unsigned)resolution; dim.stride = (unsigned)resolution*resolution;
	arrayMsg.layout.dim.push_back(dim);
	dim.label = "z"; dim.size = (unsigned)resolution; dim.stride = (unsigned)resolution;
	arrayMsg.layout.dim.push_back(dim);

	int emptyCount = 0;
	int fullCount = 0;
	int unknownCount = 0;
	for (int i = 0; i < resolution; ++i)
	{
		float x = min.x() + (max.x() - min.x())*(i/static_cast<float>(resolution-1));
		for (int j = 0; j < resolution; ++j)
		{
			float y = min.y() + (max.y() - min.y())*(j/static_cast<float>(resolution-1));
			for (int k = 0; k < resolution; ++k)
			{
				float z = min.z() + (max.z() - min.z())*(k/static_cast<float>(resolution-1));

				Eigen::Vector3f queryPoint = worldTcamera * Eigen::Vector3f(x, y, z);

				om::OcTreeNode* node = octree->search(queryPoint.x(), queryPoint.y(), queryPoint.z());
				if (!node)
				{
					//This cell is unknown
					unknownCount++;
					arrayMsg.data.push_back(0);
				}
				else
				{
					om::OcTreeNode* local_node = subtree->search(queryPoint.x(), queryPoint.y(), queryPoint.z());
					bool existsInLocalTree = ((local_node) ? (local_node->getOccupancy()>0.5) : false);

					double cellValue = node->getOccupancy() - 0.5;

//					// Temp code for testing features
//					for (unsigned int d = 0; d < octree->getTreeDepth(); ++d)
//					{
//						om::OcTreeKey key = octree->coordToKey(x, y, z, d);
//						octree->search(key, d);
//						octree->search(x, y, z, d);
//						octree->getTreeType();
//						octree->getNodeSize(d);
//						om::point3d p = octree->keyToCoord(key, d);
//					}

					if (cellValue > 0 && existsInLocalTree)
					{
						fullCount++;
						arrayMsg.data.push_back(1);
					}
					else if (cellValue < 0)
					{
						emptyCount++;
						arrayMsg.data.push_back(-1);
					}
					else
					{
//						std::cerr << "Uncertain value at " << x << ", " << y << ", " << z << std::endl;
						arrayMsg.data.push_back(0);
					}
				}
			}
		}
	}
	std::cerr << "Results are " << fullCount<< ", " << emptyCount << ", " << unknownCount << std::endl;
	if (fullCount > 0)
		localMapPub.publish(arrayMsg);
}

// Point cloud utilities
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/core/null_deleter.hpp>
//#include <abstract.h>

using PointT = pcl::PointXYZRGB;

std::shared_ptr<tf::TransformListener> listener;

template <typename PointT>
void getAABB(const pcl::PointCloud<PointT>& members, Eigen::Vector3f& min, Eigen::Vector3f& max)
{
	const int DIM = 3;
	for (int d=0; d < DIM; ++d)
	{
		min[d] = std::numeric_limits<float>::infinity();
		max[d] = -std::numeric_limits<float>::infinity();
	}

	for (const PointT& p : members)
	{
		for (int d=0; d < DIM; ++d)
		{
			min[d] = std::min(min[d], p.getVector3fMap()[d]);
			max[d] = std::max(max[d], p.getVector3fMap()[d]);
		}
	}
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
	// Get Octree
	auto octreeInfo = mapClient->getOctree();
	std::shared_ptr<om::OcTree> octree = octreeInfo.first;
	std::string globalFrame = octreeInfo.second;
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
	pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>());
	// Coordinates in table ("global") frame
	Eigen::Vector4f maxExtent(0.5, 1, 1, 1);
	Eigen::Vector4f minExtent(-0.5, -1, -0.25, 1);

	tf::StampedTransform tableFrameInCameraCoordinates;
	listener->lookupTransform(globalFrame, cloudMsg->header.frame_id, ros::Time(0), tableFrameInCameraCoordinates);
	Eigen::Affine3d worldTcamera;
	tf::transformTFToEigen(tableFrameInCameraCoordinates, worldTcamera);

	pcl::CropBox<PointT> boxFilter;
	boxFilter.setMin(minExtent);
	boxFilter.setMax(maxExtent);
	boxFilter.setTransform(worldTcamera.cast<float>());
	boxFilter.setInputCloud(cloud);
	boxFilter.filter(*cropped_cloud);

	std::cerr << "Filtered points: " << cropped_cloud->size() << std::endl;

	if (cropped_cloud->empty())
	{
		ROS_WARN("Filtered cloud contains no points.");
		return;
	}

	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;//pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01); //octree->getResolution();

	seg.setInputCloud (cropped_cloud);
	seg.segment (*table_inliers, *coefficients);

	if (table_inliers->indices.empty())
	{
		ROS_WARN("Could not estimate a planar model for the given scene.");
		return;
	}

	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(cropped_cloud);
	extractor.setIndices(table_inliers);
	extractor.setNegative(true);

	pcl::PointCloud<PointT>::Ptr non_table_cloud(new pcl::PointCloud<PointT>());
	extractor.filter(*non_table_cloud);

	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud(non_table_cloud);

	std::vector<pcl::PointIndices> clusters;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.02); // 2cm
	ec.setMinClusterSize (100);
	ec.setMaxClusterSize (25000);
	ec.setSearchMethod (tree);
	ec.setInputCloud (non_table_cloud);
	ec.extract (clusters);

	for (const pcl::PointIndices& cluster : clusters)
	{
		pcl::ExtractIndices<PointT> cluster_extractor;
		cluster_extractor.setInputCloud(non_table_cloud);
		cluster_extractor.setIndices(pcl::PointIndices::ConstPtr(&cluster, boost::null_deleter()));
		cluster_extractor.setNegative(false);

		pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
		cluster_extractor.filter(*cluster_cloud);

		// Compute bounding box
//		PointT p; p.getVector3fMap()
		Eigen::Vector3f min, max;
		getAABB(*cluster_cloud, min, max);
		Eigen::Vector3f sides = max-min;
		float max_len = sides.array().maxCoeff();

		for (int d=0; d < 3; ++d)
		{
			float delta = max_len - sides[d];
			assert(delta > -1e-9);
			if (delta > 0)
			{
				max[d] += delta/2.0;
				min[d] -= delta/2.0;
			}
		}

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
		getOccupancy(min, max, worldTcamera.cast<float>(), octree, subtree);
	}
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "shape_completion_manager");
	ros::NodeHandle nh;

	listener = std::make_shared<tf::TransformListener>();

	localMapPub = nh.advertise<std_msgs::ByteMultiArray>("local_occupancy", 1, true);
	mapClient = std::make_shared<OctreeRetriever>(nh);

	ros::Subscriber sub = nh.subscribe ("kinect2_victor_head/qhd/points", 1, cloud_cb);

//	getOccupancy(om::point3d(-0.1f, -0.1f, -0.1f), om::point3d(0.1f, 0.1f, 0.1f));

//	ros::spinOnce();
	ros::spin();

	return 0;
}