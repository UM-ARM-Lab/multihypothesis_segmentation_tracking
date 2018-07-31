//
// Created by arprice on 7/31/18.
//

#include "mps_voxels/pointcloud_utils.h"

// Cropping
#include <pcl/filters/crop_box.h>

// Segmentation
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>

#include <ros/console.h>

#include <boost/core/null_deleter.hpp>

pcl::PointCloud<PointT>::Ptr crop(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const Eigen::Vector4f& minExtent,
	const Eigen::Vector4f& maxExtent,
	const Eigen::Affine3d& worldTcamera)
{
	pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>());
	pcl::CropBox<PointT> boxFilter;
	boxFilter.setMin(minExtent);
	boxFilter.setMax(maxExtent);
	boxFilter.setTransform(worldTcamera.cast<float>());
	boxFilter.setInputCloud(cloud);
	boxFilter.filter(*cropped_cloud);

	return cropped_cloud;
}

std::vector<pcl::PointCloud<PointT>::Ptr> segment(
	pcl::PointCloud<PointT>::Ptr& cloud)
{
	std::vector<pcl::PointCloud<PointT>::Ptr> segments;

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

	seg.setInputCloud (cloud);
	seg.segment (*table_inliers, *coefficients);

	if (table_inliers->indices.empty())
	{
		ROS_WARN("Could not estimate a planar model for the given scene.");
		return segments;
	}

	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(cloud);
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

		segments.push_back(cluster_cloud);
	}

	return segments;
}