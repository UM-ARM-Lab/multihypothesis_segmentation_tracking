//
// Created by arprice on 7/3/18.
//

// Point cloud utilities
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

// Octree utilities
#include "mps_voxels/octree_utils.h"
//#include <octomap/octomap.h>
//#include <octomap_msgs/conversions.h>
//#include <octomap_msgs/GetOctomap.h>

#include <ros/ros.h>


namespace om = octomap;
using namespace mps;

using PointT = pcl::PointXYZRGB;

std::shared_ptr<OctreeRetriever> mapClient;
ros::Publisher clusterPub;
std::shared_ptr<tf::TransformListener> listener;
//sensor_msgs::PointCloud2 latestCloud;

double distance(const PointT& a, const PointT& b)
{
	return (a.getRGBVector3i() - b.getRGBVector3i()).cast<float>().norm();
}

PointT average(const std::vector<PointT>& cluster)
{
	Eigen::Vector3f xyzAccumulator = Eigen::Vector3f::Zero();
	Eigen::Vector3f rgbAccumulator = Eigen::Vector3f::Zero();
	for (const PointT& pt : cluster)
	{
		xyzAccumulator += pt.getVector3fMap().cast<float>();
		rgbAccumulator += pt.getRGBVector3i().cast<float>();
	}
	xyzAccumulator /= static_cast<float>(cluster.size());
	rgbAccumulator /= static_cast<float>(cluster.size());

	PointT center;
	center.r = static_cast<uint8_t>(rgbAccumulator[0]);
	center.g = static_cast<uint8_t>(rgbAccumulator[1]);
	center.b = static_cast<uint8_t>(rgbAccumulator[2]);

	return center;
}

template <typename Container, typename T>
struct ClusterProperties
{
	const Container& pts;
	std::vector<T> centers;
	std::vector<size_t> assignments;           // points -> clusters
	std::vector<std::vector<size_t>> clusters; // clusters -> points
};

template <typename Container, typename T>
ClusterProperties<Container, T> kMeans(const Container& pts, const int k, const int niters = 100)//, double angularWeight = 0.1)
{
	assert(pts.size() > 1);
	ClusterProperties<Container, T> props{pts, {}, {}, {}};
//	std::vector<T> centers;

	// Seed centers
	std::vector<unsigned int> indices(pts.size());
	std::iota(indices.begin(), indices.end(), 0);
	std::random_shuffle(indices.begin(), indices.end());
	for (int s = 0; s < k; ++s)
	{
		props.centers.push_back(pts[indices[s]]);
	}

	// Compute angular scale factor
//	if (angularWeight <= 0.0)
//	{
//		double maxLinDist = 0.0;
//		double maxAngDist = 0.0;
//		for (size_t i = 0; i < pts.size(); ++i)
//		{
//			for (size_t j = 0; j < pts.size(); ++j)
//			{
//				double linDist = distanceLinear(pts[i], pts[j]);
//				double angDist = distanceAngular(pts[i], pts[j]);
//
//				maxLinDist = std::max(linDist, maxLinDist);
//				maxAngDist = std::max(angDist, maxAngDist);
//			}
//		}
//		angularWeight = maxLinDist/maxAngDist;
//	}
//
//	std::cerr << "Angular Weight: " << angularWeight << std::endl;

	std::vector<std::vector<size_t>> lastClusters(props.centers.size());
	for (int iter = 0; iter < niters; ++iter)
	{
		// Compute cluster membership
		props.assignments = std::vector<size_t>(pts.size());                     // points -> clusters
		props.clusters = std::vector<std::vector<size_t>>(props.centers.size()); // clusters -> points

#pragma omp parallel for
		for (size_t p = 0; p < pts.size(); ++p)
		{
			size_t minIdx = 0;
			double minDist = std::numeric_limits<double>::infinity();

			// Find the nearest cluster
//			#pragma omp parallel for reduction(min : minDist)
			for (size_t c = 0; c < props.centers.size(); ++c)
			{
				double d = distance(props.centers[c], pts[p]);
				if (d < minDist)
				{
					minDist = d;
					minIdx = c;
				}
			}

			props.assignments[p] = minIdx;
		}

		// Reverse lookup
		for (size_t p = 0; p < pts.size(); ++p)
		{
			props.clusters[props.assignments[p]].push_back(p);
		}

		// Re-center clusters
#pragma omp parallel for
		for (size_t c = 0; c < props.centers.size(); ++c)
		{
			std::vector<T> cluster(props.clusters[c].size());
			for (size_t i = 0; i < props.clusters[c].size(); ++i)
			{
				cluster[i] = pts[props.clusters[c][i]];
			}

			if (!props.clusters[c].empty())
			{
				props.centers[c] = average(cluster);
			}
		}

		if (props.clusters == lastClusters)
		{
			std::cerr << "Clustering converged." << std::endl;
			break;
		}
		else
		{
			lastClusters = props.clusters;
		}
	}

	return props;
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloudMsg)
{
	// Do data processing here...
//	latestCloud = *cloud_msg;

//	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
//	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
//	pcl_conversions::toPCL(*cloud_msg, cloud);

	// Get Octree
	std::shared_ptr<om::OcTree> octree;
	std::string globalFrame;
	std::tie(octree, globalFrame) = mapClient->getOctree();
	if (!octree)
	{
		ROS_ERROR("Octree lookup failed.");
		return;
	}


	std::cerr << cloudMsg->header.frame_id << " -> " << globalFrame << std::endl;

	// Transform point cloud to local frame
	sensor_msgs::PointCloud2 transformedCloud;
	if (!listener->waitForTransform(globalFrame, cloudMsg->header.frame_id, ros::Time(0), ros::Duration(5.0)))
	{
		ROS_WARN_STREAM("Failed to look up transform between '" << globalFrame << "' and '" << cloudMsg->header.frame_id << "'.");
		return;
	}
	pcl_ros::transformPointCloud(globalFrame, *cloudMsg, transformedCloud, *listener);

	pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>());
	pcl::fromROSMsg(transformedCloud, *cloud);

	// Filter box
	pcl::PointCloud<PointT> filter_cloud;
	Eigen::Vector4f maxExtent(0.5, 1, 1, 1);
	Eigen::Vector4f minExtent(-0.5, -1, -0.25, 1);

	pcl::CropBox<PointT> boxFilter;
	boxFilter.setMin(minExtent);
	boxFilter.setMax(maxExtent);
	boxFilter.setInputCloud(cloud);
	boxFilter.filter(filter_cloud);

	std::cerr << "Filtered points: " << filter_cloud.size() << std::endl;

	if (filter_cloud.empty())
	{
		ROS_WARN("Filtered cloud contains no points.");
		return;
	}

	ClusterProperties<pcl::PointCloud<PointT>, PointT> clusters = kMeans<pcl::PointCloud<PointT>, PointT>(filter_cloud, 8);

	// Recolor cloud by exemplars (cluster centers)
	pcl::PointCloud<PointT>::Ptr cluster_cloud(new pcl::PointCloud<PointT>());
	for (size_t idx = 0; idx < filter_cloud.size(); ++idx)
	{
		PointT pt = filter_cloud[idx];
		pt.rgba = clusters.centers[clusters.assignments[idx]].rgba;
		cluster_cloud->push_back(pt);
	}
	sensor_msgs::PointCloud2 cloud_out;
	pcl::toROSMsg(*cluster_cloud, cloud_out);
	cloud_out.header.frame_id = globalFrame;
	cloud_out.header.stamp = cloudMsg->header.stamp;
	clusterPub.publish(cloud_out);

//	std::map<om::OcTreeKey, std::vector<int>> cellMembers;
//	std::map<om::OcTreeNode*, std::vector<int>> cellMembers;

	// cell, label, count
	std::map<om::OcTreeNode*, std::map<int, int>> cellMembers;
	std::map<om::OcTreeNode*, int> cellCounts;

	for (size_t idx = 0; idx < filter_cloud.size(); ++idx)
	{
		const PointT& pt = filter_cloud[idx];

		// Get tree nodes this point participates in
		for (unsigned int d = 0; d < octree->getTreeDepth(); ++d)
		{
//			om::OcTreeKey key = octree->coordToKey(pt.x, pt.y, pt.z, d);
//			cellMembers[key].push_back(idx);

			om::OcTreeNode* node = octree->search(pt.x, pt.y, pt.z, d);
			if (!node)
			{
				ROS_ERROR_STREAM_ONCE("Octree does not contain (" << pt.x << ", " << pt.y << ", " << pt.z << ")!");
				continue;
			}
			cellMembers[node][clusters.assignments[idx]]++;
			cellCounts[node]++;
		}
	}

	double entropy = 0;
	for (const auto& cell : cellMembers)
	{
		double cellCount = cellCounts[cell.first];
		for (const auto& label : cell.second)
		{
			double p = label.second / cellCount;
			entropy -= p * log(p);
		}
	}

	std::cerr << "Entropy: " << entropy << std::endl;
}



int main(int argc, char* argv[])
{
	ros::init(argc, argv, "entropy_estimator");
	ros::NodeHandle nh;

	listener = std::make_shared<tf::TransformListener>();

	clusterPub = nh.advertise<sensor_msgs::PointCloud2>("cluster_points", 1, true);
	mapClient = std::make_shared<OctreeRetriever>(nh);

	ros::Subscriber sub = nh.subscribe ("kinect2_roof/qhd/points", 1, cloud_cb);

	ros::spin();

	return 0;
}