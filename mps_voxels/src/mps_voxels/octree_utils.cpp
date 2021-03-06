/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mps_voxels/octree_utils.h"
#include "mps_voxels/colormap.h"
#include "mps_voxels/util/assert.h"
#include "mps_voxels/ROI.h"
#include <mps_shape_completion_msgs/CompleteShape.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <tf_conversions/tf_eigen.h>

namespace mps
{

namespace om = octomap;

bool isSpeckleNode(const octomap::OcTreeKey& nKey, const octomap::OcTree* octree)
{
	octomap::OcTreeKey key;
	bool neighborFound = false;
	for (key[2] = nKey[2] - 1; !neighborFound && key[2] <= nKey[2] + 1; ++key[2]){
		for (key[1] = nKey[1] - 1; !neighborFound && key[1] <= nKey[1] + 1; ++key[1]){
			for (key[0] = nKey[0] - 1; !neighborFound && key[0] <= nKey[0] + 1; ++key[0]){
				if (key != nKey){
					octomap::OcTreeNode* node = octree->search(key);
					if (node && octree->isNodeOccupied(node)){
						// we have a neighbor => break!
						neighborFound = true;
					}
				}
			}
		}
	}

	return !neighborFound;
}

void decayMemory(octomap::OcTree* octree, const octomap::point3d& cameraOrigin, const float alphaVisible, const float alphaHidden)
{
	// Decay to just below the occupancy threshold
	const float decayTarget = 0.8 * octree->getOccupancyThres();
	MPS_ASSERT(decayTarget > 0.0);
	float alpha = alphaHidden;

	std::vector<octomap::OcTreeKey> toDelete;

	for (octomap::OcTree::iterator it = octree->begin(octree->getTreeDepth()),
		     end = octree->end(); it != end; ++it)
	{
		if (octree->isNodeOccupied(*it))
		{
			const auto& pt_world = it.getCoordinate();

			octomap::point3d collision;
			bool hit = octree->castRay(cameraOrigin, pt_world-cameraOrigin, collision, true);
			MPS_ASSERT(hit);

			float hitDist = pt_world.distance(collision);

			if (hitDist <= 2.0*octree->getResolution())
			{
				alpha = alphaVisible;
			}
			else
			{
				alpha = alphaHidden;
			}

			it->setValue(alpha * it->getValue() + (1.0f-alpha)*decayTarget);
		}

		if (fabs(it->getValue()-decayTarget) < 0.2)
		{
			toDelete.push_back(it.getKey());
		}
	}

	for (const octomap::OcTreeKey& key : toDelete) { octree->deleteNode(key, octree->getTreeDepth()); }

	octree->updateInnerOccupancy();
	octree->prune();

}


OctreeRetriever::OctreeRetriever(ros::NodeHandle& nh)
{
	mapClient = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
	if (!mapClient.waitForExistence(ros::Duration(3)))
	{
		ROS_WARN("Map server not connected.");
	}
}

std::pair<std::shared_ptr<om::OcTree>, std::string> OctreeRetriever::getOctree()
{
	octomap_msgs::GetOctomapRequest req;
	octomap_msgs::GetOctomapResponse resp;
	bool callSucceeded = mapClient.call(req, resp);
	if (!callSucceeded)
	{
		ROS_ERROR("Unable to call Octomap service.");
		return {std::shared_ptr<om::OcTree>(), resp.map.header.frame_id};
	}

	std::shared_ptr<octomap::AbstractOcTree> abstractTree(octomap_msgs::msgToMap(resp.map));
	std::shared_ptr<om::OcTree> octree = std::dynamic_pointer_cast<om::OcTree>(abstractTree);

	if (!octree)
	{
		ROS_ERROR("Unable to downcast abstract octree to concrete tree.");
		return {std::shared_ptr<om::OcTree>(), resp.map.header.frame_id};
	}

	return {octree, resp.map.header.frame_id};
}

VoxelCompleter::VoxelCompleter(ros::NodeHandle& nh)
{
	completionClient = nh.serviceClient<mps_shape_completion_msgs::CompleteShape>("/complete_shape");
	if (!completionClient.waitForExistence(ros::Duration(3)))
	{
		ROS_WARN("Shape completion server not connected.");
	}
}

void VoxelCompleter::completeShape(
	const Eigen::Vector3d& min,
	const Eigen::Vector3d& max,
	const Eigen::Affine3f& worldTcamera,
	octomap::OcTree* octree,
	octomap::OcTree* subtree,
	bool updateMainTree,
	const int resolution) const
{
	MPS_ASSERT(resolution > 0);
	MPS_ASSERT(max.x() > min.x());
	MPS_ASSERT(max.y() > min.y());
	MPS_ASSERT(max.z() > min.z());

	std::cerr << octree->getNumLeafNodes() << ", " << octree->getResolution() << std::endl;
	if (subtree)
	{
		std::cerr << subtree->getNumLeafNodes() << ", " << subtree->getResolution() << std::endl;
	}

	mps_shape_completion_msgs::CompleteShapeRequest req;
//	std_msgs::ByteMultiArray arrayMsg;
	std_msgs::MultiArrayDimension dim;

	dim.label = "x"; dim.size = (unsigned)resolution; dim.stride = (unsigned)resolution*resolution*resolution;
	req.observation.layout.dim.push_back(dim);
	dim.label = "y"; dim.size = (unsigned)resolution; dim.stride = (unsigned)resolution*resolution;
	req.observation.layout.dim.push_back(dim);
	dim.label = "z"; dim.size = (unsigned)resolution; dim.stride = (unsigned)resolution;
	req.observation.layout.dim.push_back(dim);

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

				// World frame is table surface, so no points below it are allowed
				if (queryPoint.z() < 0)
				{
					req.observation.data.push_back(-1); // Cell is known free (not part of object)
					continue;
				}

				om::OcTreeNode* node = octree->search(queryPoint.x(), queryPoint.y(), queryPoint.z());
				if (!node)
				{
					//This cell is unknown
					unknownCount++;
					req.observation.data.push_back(0); // Cell is unknown
				}
				else
				{
					om::OcTreeNode* local_node = subtree->search(queryPoint.x(), queryPoint.y(), queryPoint.z());
					bool existsInLocalTree = ((local_node) ? (local_node->getOccupancy()>0.5) : false);

					double cellValue = node->getOccupancy();

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

					if (cellValue > 0.5 && existsInLocalTree)
					{
						fullCount++;
						req.observation.data.push_back(1); // Cell is known occupied
					}
					else
					{
						emptyCount++;
						req.observation.data.push_back(-1); // Cell is known free
					}
//					else if (cellValue < 0.5)
//					{
//						emptyCount++;
//						req.observation.data.push_back(-1);
//					}
//					else
//					{
////						std::cerr << "Uncertain value at " << x << ", " << y << ", " << z << std::endl;
//						req.observation.data.push_back(0);
//					}
				}
			}
		}
	}
	std::cerr << "Results are " << fullCount<< ", " << emptyCount << ", " << unknownCount << std::endl;
	if (fullCount == 0)
	{
		ROS_WARN_STREAM("No occupied cells were detected in the region.");
		return;
	}
	mps_shape_completion_msgs::CompleteShapeResponse res;
	bool succeeded = completionClient.call(req, res);
	if (!succeeded)
	{
		ROS_ERROR_STREAM("Shape completion call failed.");
		return;
	}

	std::vector<octomap::OcTree*> treesToUpdate{subtree};
	if (updateMainTree) { treesToUpdate.push_back(octree); }

	const float OCCUPANCY_THRESHOLD = std::nextafter(0.0f, std::numeric_limits<float>::max());

	for (int i = 0; i < resolution; ++i)
	{
		float x = min.x()+(max.x()-min.x())*(i/static_cast<float>(resolution-1));
		for (int j = 0; j<resolution; ++j)
		{
			float y = min.y()+(max.y()-min.y())*(j/static_cast<float>(resolution-1));
			for (int k = 0; k<resolution; ++k)
			{
				float z = min.z()+(max.z()-min.z())*(k/static_cast<float>(resolution-1));

				int idx = i*resolution*resolution + j*resolution + k;
				bool predictedFilled = res.hypothesis.data[idx] >= OCCUPANCY_THRESHOLD;

				if (predictedFilled)
				{
					Eigen::Vector3f queryPoint = worldTcamera*Eigen::Vector3f(x, y, z);

					if (queryPoint.z() < 0) { continue; }

					for (auto& tree : treesToUpdate)
					{
						om::OcTreeNode* node = tree->search(queryPoint.x(), queryPoint.y(), queryPoint.z());
						if (!node)
						{
							node = tree->setNodeValue(queryPoint.x(), queryPoint.y(), queryPoint.z(), octomap::logodds(res.hypothesis.data[idx]), false);
						}
						else
						{
							node->setValue(octomap::logodds(res.hypothesis.data[idx]));
						}
					}
				}
			}
		}
	}
}

octomap::point3d_collection getPoints(const octomap::OcTree* tree)
{
	octomap::point3d_collection pts;
	pts.reserve(tree->size()/2); // Guess that the octree is half-full
	for (octomap::OcTree::iterator it = tree->begin(tree->getTreeDepth()),
		     end = tree->end(); it != end; ++it)
	{
		if (tree->isNodeOccupied(*it))
		{
//			unsigned idx = it.getDepth();

			pts.emplace_back(it.getCoordinate());
		}
	}
	pts.shrink_to_fit();
	return pts;
}

octomap::point3d_collection
getExteriorPoints(const octomap::OcTree* tree)
{
	octomap::point3d_collection pts;
	pts.reserve(tree->size()/2); // Guess that the octree is half-full

	for (octomap::OcTree::iterator it = tree->begin(tree->getTreeDepth()),
		     end = tree->end(); it != end; ++it)
	{
		if (tree->isNodeOccupied(*it))
		{
			const unsigned idx = it.getDepth();
			const auto halfRes = static_cast<float>(tree->getNodeSize(idx)/2.0);

			const auto& center = it.getCoordinate();
			octomap::point3d offset(halfRes, halfRes, halfRes);

			octomap::point3d min = center - offset;
			octomap::point3d max = center + offset;

			// TODO: The main version of this is in ROI.h, but only supports [], not (). enable_if
			// There will be 2^DIM corners to deal with
			const unsigned nCorners = (1u << 3u);
			for (unsigned perm = 0; perm < nCorners; ++perm)
			{
				octomap::point3d pt;
				for (unsigned d = 0; d < 3; ++d)
				{
					pt(d) = (perm & (1u<<d)) ? min(d) : max(d);
				}
				pts.emplace_back(pt);
			}
		}
	}
	pts.shrink_to_fit();
	return pts;
}

std::pair<octomap::point3d_collection, std::shared_ptr<octomap::OcTree>> getOcclusionsInFOV(
	octomap::OcTree* octree,
	const image_geometry::PinholeCameraModel& cameraModel,
	const Eigen::Affine3d& cameraTtable,
	const Eigen::Vector3d& minExtent,
	const Eigen::Vector3d& maxExtent)
{
//	std::vector<Eigen::Vector3f>
	octomap::point3d_collection occluded_pts;
	std::shared_ptr<octomap::OcTree> occlusionTree(octree->create());

	const float resolution = static_cast<float>(octree->getResolution());
	const unsigned int d = octree->getTreeDepth();

	Eigen::Affine3f tableTcamera = cameraTtable.inverse(Eigen::Isometry).cast<float>();
	octomap::point3d cameraOrigin(tableTcamera.translation().x(), tableTcamera.translation().y(), tableTcamera.translation().z());

#if defined(_OPENMP)
	int nOuterSteps = static_cast<int>((maxExtent.x()-minExtent.x())/resolution);
	#pragma omp parallel for
	for (int i = 0; i < nOuterSteps; ++i)
	{
		float ix = minExtent.x() + (i*resolution);
#else
	for (float ix = minExtent.x(); ix <= maxExtent.x(); ix += resolution)
	{
#endif
		for (float iy = minExtent.y(); iy <= maxExtent.y(); iy += resolution)
		{
			for (float iz = minExtent.z(); iz <= maxExtent.z(); iz += resolution)
			{
				Eigen::Vector3d p = cameraTtable * Eigen::Vector3d(ix, iy, iz);
				cv::Point3d worldPt_camera(p.x(), p.y(), p.z());
				cv::Point2d imgPt = cameraModel.project3dToPixel(worldPt_camera);
				const int x_buffer = cameraModel.cameraInfo().width/8;
				const int y_buffer = cameraModel.cameraInfo().height/8;
				if (imgPt.x > x_buffer && imgPt.x < cameraModel.cameraInfo().width - x_buffer
				    && imgPt.y > y_buffer && imgPt.y < cameraModel.cameraInfo().height - y_buffer)
				{
					if (!octree->inBBX(octomap::point3d(ix, iy, iz))) { continue; }

					if (!octree->search(ix, iy, iz))
					{
						//This cell is unknown
						// Snap from (ix,iy,iz) to octree coordinate grid
						octomap::point3d coord = octree->keyToCoord(octree->coordToKey(ix, iy, iz, d), d);

						octomap::point3d ray = coord-cameraOrigin;
						octomap::point3d collision;
						bool collided = octree->castRay(cameraOrigin, ray, collision, false);

						#pragma omp critical
						{
							if (collided && (ray.norm_sq()>(collision-cameraOrigin).norm_sq()+octree->getResolution()))
							{
								occluded_pts.push_back(coord);
								occlusionTree->setNodeValue(coord, std::numeric_limits<float>::infinity(), true);
							}
							else
							{
								octree->updateNode(coord, octree->getProbMissLog(), true);
							}
						}
					}
				}
			}
		}
	}
	occlusionTree->updateInnerOccupancy();

	return { occluded_pts, occlusionTree };
}

visualization_msgs::MarkerArray visualizeOctree(octomap::OcTree* tree, const std::string& globalFrame, const std_msgs::ColorRGBA* base_color)
{
	visualization_msgs::MarkerArray occupiedNodesVis;
	occupiedNodesVis.markers.resize(tree->getTreeDepth()+1);

	for (octomap::OcTree::iterator it = tree->begin(tree->getTreeDepth()),
		     end = tree->end(); it != end; ++it)
	{
		if ((!base_color && tree->isNodeOccupied(*it)) || (base_color && it->getOccupancy() > tree->getOccupancyThres()))
		{
			unsigned idx = it.getDepth();

			geometry_msgs::Point cubeCenter;
			cubeCenter.x = it.getX();
			cubeCenter.y = it.getY();
			cubeCenter.z = it.getZ();

			occupiedNodesVis.markers[idx].points.push_back(cubeCenter);

			// Colors
			std_msgs::ColorRGBA color;
			color.a = 1.0;
			if (base_color)
			{
				float prob = (float)it->getOccupancy();
				color.r = prob * base_color->r;
				color.g = prob * base_color->g;
				color.b = prob * base_color->b;
			}
			else
			{
				colormap(igl::parula_cm, (float)((it->getOccupancy()-tree->getOccupancyThres())*2.0), color.r, color.g, color.b);
			}
			occupiedNodesVis.markers[idx].colors.push_back(color);
		}
	}

	for (unsigned i = 0; i<occupiedNodesVis.markers.size(); ++i)
	{
		double size = tree->getNodeSize(i);

		occupiedNodesVis.markers[i].header.frame_id = globalFrame;
		occupiedNodesVis.markers[i].header.stamp = ros::Time::now();
		occupiedNodesVis.markers[i].ns = "map" + std::to_string(i);//"occlusion";
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
		occupiedNodesVis.markers[i].pose.orientation.w = 1;

		if (occupiedNodesVis.markers[i].points.size()>0)
			occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
		else
			occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
	}

		return occupiedNodesVis;
}

}
