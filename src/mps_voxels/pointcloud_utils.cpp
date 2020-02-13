//
// Created by arprice on 7/31/18.
//

#include "mps_voxels/pointcloud_utils.h"
#include "mps_voxels/util/assert.h"

// Cropping
#include <pcl/filters/crop_box.h>

// Segmentation
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>

#include <pcl/search/impl/search.hpp> // needed for kd-tree?

#include <ros/console.h>

#include <boost/core/null_deleter.hpp>

namespace mps
{

pcl::PointCloud<PointT>::Ptr filterInCameraFrame(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const double zMin,
	const double zMax,
	const int k)
{
	pcl::PointCloud<PointT>::Ptr cloud_trimmed (new pcl::PointCloud<PointT>);

	pcl::PassThrough<PointT> pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("z");
	pass.setFilterLimits(zMin, zMax);
	//pass.setFilterLimitsNegative (true);
	pass.filter (*cloud_trimmed);

	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (cloud_trimmed);
	sor.setMeanK (k);
	sor.setStddevMulThresh (1.0);
	sor.filter (*cloud_filtered);

	return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr cropInCameraFrame(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const Eigen::Vector4f& minExtent,
	const Eigen::Vector4f& maxExtent,
	const Eigen::Affine3d& worldTcamera)
{
	for (int i = 0; i < 3; ++i) { MPS_ASSERT(minExtent[i] < maxExtent[i]); }

	pcl::PointCloud<PointT>::Ptr cropped_cloud(new pcl::PointCloud<PointT>());
	pcl::CropBox<PointT> boxFilter;
	boxFilter.setMin(minExtent);
	boxFilter.setMax(maxExtent);
	boxFilter.setTransform(worldTcamera.cast<float>());
	boxFilter.setInputCloud(cloud);
	boxFilter.filter(*cropped_cloud);

	return cropped_cloud;
}

pcl::PointCloud<PointT>::Ptr filterOutliers(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const int k,
	const double stddev)
{
	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);

	// Create the filtering object
	pcl::StatisticalOutlierRemoval<PointT> sor;
	sor.setInputCloud (cloud);
	sor.setMeanK (k);
	sor.setStddevMulThresh (stddev);
	sor.filter (*cloud_filtered);

	return cloud_filtered;
}

pcl::PointCloud<PointT>::Ptr filterPlane(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const double distanceThreshold,
	const Eigen::Vector3f& normal)
{
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr table_inliers (new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation<PointT> seg;//pcl::SACSegmentation<pcl::PointXYZ> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (distanceThreshold); //octree->getResolution();
	MPS_ASSERT(normal.norm() > 0.1);
	if (normal.norm() > 0.1)
	{
		seg.setAxis(normal);
		seg.setEpsAngle(0.2);
	}

	seg.setInputCloud (cloud);
	seg.segment (*table_inliers, *coefficients);

	if (table_inliers->indices.empty())
	{
		ROS_WARN("Could not estimate a planar model for the given scene.");
		return pcl::PointCloud<PointT>::Ptr();
	}

	pcl::ExtractIndices<PointT> extractor;
	extractor.setInputCloud(cloud);
	extractor.setIndices(table_inliers);
	extractor.setNegative(true);

	pcl::PointCloud<PointT>::Ptr non_table_cloud(new pcl::PointCloud<PointT>());
	extractor.filter(*non_table_cloud);

	return non_table_cloud;
}

pcl::PointCloud<PointT>::Ptr filterSmallClusters(
	pcl::PointCloud<PointT>::Ptr& cloud,
	const int clusterThreshold,
	const int clusterDistance)
{
	// TODO: Doesn't work...
	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud(cloud);

	std::vector<pcl::PointIndices> largeClusters;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (clusterDistance);
	ec.setMinClusterSize (clusterThreshold);
	ec.setMaxClusterSize (cloud->size());
	ec.setSearchMethod (tree);
	ec.setInputCloud (cloud);
	ec.extract (largeClusters);

	unsigned removeN = 0;
	for (const pcl::PointIndices& cluster : largeClusters)
	{
		removeN += cluster.indices.size();
	}

	pcl::PointIndices toKeep;
	toKeep.indices.reserve(removeN);
	for (const pcl::PointIndices& cluster : largeClusters)
	{
		toKeep.indices.insert(toKeep.indices.end(), cluster.indices.begin(), cluster.indices.end());
	}

	pcl::ExtractIndices<PointT> cluster_extractor;
	cluster_extractor.setInputCloud(cloud);
	cluster_extractor.setIndices(pcl::PointIndices::ConstPtr(&toKeep, boost::null_deleter()));
	cluster_extractor.setNegative(false);

	pcl::PointCloud<PointT>::Ptr cloud_filtered (new pcl::PointCloud<PointT>);
	cluster_extractor.filter(*cloud_filtered);

	return cloud_filtered;
}

std::vector<pcl::PointCloud<PointT>::Ptr> segment(
	pcl::PointCloud<PointT>::Ptr& cloud)
{
	std::vector<pcl::PointCloud<PointT>::Ptr> segments;

	pcl::PointCloud<PointT>::Ptr non_table_cloud = filterPlane(cloud);
	if (!non_table_cloud || non_table_cloud->empty())
	{
		return segments;
	}

	pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
	tree->setInputCloud(non_table_cloud);

	std::vector<pcl::PointIndices> clusters;
	pcl::EuclideanClusterExtraction<PointT> ec;
	ec.setClusterTolerance (0.03); // 3cm
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

void getAABB(const pcl::PointCloud<PointT>& members, Eigen::Vector3f& min, Eigen::Vector3f& max)
{
	const int DIM = 3;
	for (int d=0; d < DIM; ++d)
	{
		min[d] = std::numeric_limits<float>::infinity();
		max[d] = -std::numeric_limits<float>::infinity();
	}

//	#pragma omp parallel for reduction(max : max) reduction(min : min)
	for (const PointT& p : members)
	{
		for (int d=0; d < DIM; ++d)
		{
			min[d] = std::min(min[d], p.getVector3fMap()[d]);
			max[d] = std::max(max[d], p.getVector3fMap()[d]);
		}
	}
}

void getBoundingCube(const pcl::PointCloud<PointT>& members, Eigen::Vector3f& min, Eigen::Vector3f& max)
{
	getAABB(members, min, max);
	Eigen::Vector3f sides = max-min;
	float max_len = sides.array().maxCoeff();

	for (int d = 0; d<3; ++d)
	{
		float delta = max_len-sides[d];
		assert(delta>-1e-9);
		if (delta>0)
		{
			max[d] += delta/2.0;
			min[d] -= delta/2.0;
		}
	}
}

}
