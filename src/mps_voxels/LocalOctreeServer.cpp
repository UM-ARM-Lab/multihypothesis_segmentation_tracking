//
// Created by arprice on 9/10/18.
//

#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/util/assert.h"

#include <octomap/Pointcloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>

namespace mps
{

/*namespace octomap
{

bool computeRayKeys(OcTree* tree,
                    const point3d& origin,
                    const point3d& end,
                    KeyRay& ray)
{

	// see "A Faster Voxel Traversal Algorithm for Ray Tracing" by Amanatides & Woo
	// basically: DDA in 3D

	ray.reset();

	OcTreeKey key_origin, key_end;
	key_origin = tree->coordToKey(origin);
	key_end = tree->coordToKey(end);


	if (key_origin == key_end)
		return true; // same tree cell, we're done.

	ray.addKey(key_origin);

	// Initialization phase -------------------------------------------------------

	point3d direction = (end - origin);
	float length = (float) direction.norm();
	direction /= length; // normalize vector

	int    step[3];
	double tMax[3];
	double tDelta[3];

	OcTreeKey current_key = key_origin;

	for(unsigned int i=0; i < 3; ++i) {
		// compute step direction
		if (direction(i) > 0.0) step[i] =  1;
		else if (direction(i) < 0.0)   step[i] = -1;
		else step[i] = 0;

		// compute tMax, tDelta
		if (step[i] != 0) {
			// corner point of voxel (in direction of ray)
			double voxelBorder = tree->keyToCoord(current_key[i]);
			voxelBorder += (float) (step[i] * tree->getResolution() * 0.5);

			tMax[i] = ( voxelBorder - origin(i) ) / direction(i);
			tDelta[i] = tree->getResolution() / fabs( direction(i) );
		}
		else {
			tMax[i] =  std::numeric_limits<double>::max( );
			tDelta[i] = std::numeric_limits<double>::max( );
		}
	}

	// Incremental phase  ---------------------------------------------------------

	bool done = false;
	while (!done) {

		unsigned int dim;

		// find minimum tMax:
		if (tMax[0] < tMax[1]){
			if (tMax[0] < tMax[2]) dim = 0;
			else                   dim = 2;
		}
		else {
			if (tMax[1] < tMax[2]) dim = 1;
			else                   dim = 2;
		}

		// advance in direction "dim"
		current_key[dim] += step[dim];
		tMax[dim] += tDelta[dim];

		assert (current_key[dim] < 2*tree->tree_max_val);

		// reached endpoint, key equv?
		if (current_key == key_end) {
			done = true;
			break;
		}
		else if (!tree->inBBX(current_key))
		{
			done = true;
			break;
		}
		else {

			// reached endpoint world coords?
			// dist_from_origin now contains the length of the ray when traveled until the border of the current voxel
			double dist_from_origin = std::min(std::min(tMax[0], tMax[1]), tMax[2]);
			// if this is longer than the expected ray length, we should have already hit the voxel containing the end point with the code above (key_end).
			// However, we did not hit it due to accumulating discretization errors, so this is the point here to stop the ray as we would never reach the voxel key_end
			if (dist_from_origin > length) {
				done = true;
				break;
			}

			else {  // continue to add freespace cells
				ray.addKey(current_key);
			}
		}

		assert ( ray.size() < ray.sizeMax() - 1);

	} // end while

	return true;
}

void insertPointCloudRays(OcTree* tree, const Pointcloud& pc, const point3d& origin,
                          double maxrange, bool lazy_eval)
{
	if (pc.size()<1)
	{
		return;
	}
	std::vector<KeyRay> keyrays(omp_get_max_threads());

	#pragma omp parallel for
	for (int i = 0; i<(int) pc.size(); ++i)
	{
		const point3d& p = pc[i];
		unsigned threadIdx = 0;
		threadIdx = omp_get_thread_num();
		KeyRay* keyray = &(keyrays.at(threadIdx));

		if (computeRayKeys(tree, origin, p, *keyray))
		{
			for (KeyRay::iterator it = keyray->begin(); it!=keyray->end(); it++)
			{
				if (tree->inBBX(*it))
				{
					#pragma omp critical
					tree->updateNode(*it, false, lazy_eval); // insert freespace measurement
				}
			}
			if (tree->inBBX(p))
			{
				#pragma omp critical
				tree->updateNode(p, true, lazy_eval); // update endpoint to be occupied
			}
		}
	}
}

}*/

void insertCloudInOctree(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& worldTcamera, octomap::OcTree* tree)
{
	if (cloud->empty()) { throw std::runtime_error("Trying to insert empty cloud!"); }

	pcl::PointCloud<PointT>::Ptr transformed_cloud (new pcl::PointCloud<PointT>());
	pcl::transformPointCloud (*cloud, *transformed_cloud, worldTcamera);

	octomap::point3d min = tree->getBBXMin(), max = tree->getBBXMax();
	for (int i = 0; i < 3; ++i)
	{
		min(i) = std::min(min(i), (float)worldTcamera.translation()[i]);
		max(i) = std::max(max(i), (float)worldTcamera.translation()[i]);
	}
	setBBox(min, max, tree);

	octomap::Pointcloud pc;
	pc.reserve(transformed_cloud->size());
	for (const PointT& pt : *transformed_cloud)
	{
		pc.push_back(pt.x, pt.y, pt.z);
//		MPS_ASSERT(pt.x > min(0)); MPS_ASSERT(pt.x < max(0));
//		MPS_ASSERT(pt.y > min(1)); MPS_ASSERT(pt.y < max(1));
//		MPS_ASSERT(pt.z > min(2)); MPS_ASSERT(pt.z < max(2));
	}
//	MPS_ASSERT(pc.size() == cloud->size());
	octomap::point3d origin((float)worldTcamera.translation().x(),
	                        (float)worldTcamera.translation().y(),
	                        (float)worldTcamera.translation().z());
//	m_octree->insertPointCloud(pc, origin, -1, true, false);
//	m_octree->insertPointCloudRays(pc, origin, -1, true);

//	insertPointCloudRays(tree, pc, origin, -1, true);

//	tree->insertPointCloudRays(pc, origin, -1, true);
	tree->insertPointCloud(pc, origin, -1, false, true);
	tree->updateInnerOccupancy();

	MPS_ASSERT(tree->size() > 0);
}

LocalOctreeServer::LocalOctreeServer(const ros::NodeHandle& private_nh_)
	: m_res(0.05),
	  m_worldFrameId("/map"),
	  m_octree(nullptr)
{
	private_nh_.param("resolution", m_res, m_res);
	private_nh_.param("frame_id", m_worldFrameId, m_worldFrameId);

	m_octree = std::make_unique<OcTreeT>(m_res);


	Eigen::Vector3d min, max;
	m_octree->getMetricMin(min.x(), min.y(), min.z());
	m_octree->getMetricMax(max.x(), max.y(), max.z());
	setBBox(min, max, m_octree.get());
}

void LocalOctreeServer::insertCloud(const pcl::PointCloud<PointT>::Ptr& cloud, const Eigen::Affine3d& worldTcamera)
{
//	pcl::PointCloud<PointT>::Ptr downsampled_cloud(new pcl::PointCloud<PointT>());
//	pcl::VoxelGrid<PointT> voxelFilter;
//	voxelFilter.setInputCloud(cloud);
//	float resolution = (float)m_octree->getResolution()/4.0;
//	voxelFilter.setLeafSize(resolution, resolution, resolution);
//	voxelFilter.filter(*downsampled_cloud);

	insertCloudInOctree(cloud, worldTcamera, m_octree.get());
}

}
