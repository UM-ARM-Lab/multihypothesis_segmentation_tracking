//
// Created by arprice on 7/24/18.
//

#include "mps_voxels/octree_utils.h"
#include "mps_voxels/colormap.h"
#include <mps_msgs/CompleteShape.h>

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

#include <tf_conversions/tf_eigen.h>

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
	completionClient = nh.serviceClient<mps_msgs::CompleteShape>("/complete_shape");
	if (!completionClient.waitForExistence(ros::Duration(3)))
	{
		ROS_WARN("Shape completion server not connected.");
	}
}

void VoxelCompleter::completeShape(
	const Eigen::Vector3f& min,
	const Eigen::Vector3f& max,
	const Eigen::Affine3f& worldTcamera,
	octomap::OcTree* octree,
	octomap::OcTree* subtree,
	bool updateMainTree,
	const int resolution) const
{
	assert(resolution > 0);
	assert(max.x() > min.x());
	assert(max.y() > min.y());
	assert(max.z() > min.z());

	std::cerr << octree->getNumLeafNodes() << ", " << octree->getResolution() << std::endl;
	if (subtree)
	{
		std::cerr << subtree->getNumLeafNodes() << ", " << subtree->getResolution() << std::endl;
	}

	mps_msgs::CompleteShapeRequest req;
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

				om::OcTreeNode* node = octree->search(queryPoint.x(), queryPoint.y(), queryPoint.z());
				if (!node)
				{
					//This cell is unknown
					unknownCount++;
					req.observation.data.push_back(0);
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
						req.observation.data.push_back(1);
					}
					else if (cellValue < 0)
					{
						emptyCount++;
						req.observation.data.push_back(-1);
					}
					else
					{
//						std::cerr << "Uncertain value at " << x << ", " << y << ", " << z << std::endl;
						req.observation.data.push_back(0);
					}
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
	mps_msgs::CompleteShapeResponse res;
	bool succeeded = completionClient.call(req, res);
	if (!succeeded)
	{
		ROS_ERROR_STREAM("Shape completion call failed.");
		return;
	}

	std::vector<octomap::OcTree*> treesToUpdate{subtree};
	if (updateMainTree) { treesToUpdate.push_back(octree); }

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
				bool predictedFilled = res.hypothesis.data[idx] > 0;

				if (predictedFilled)
				{
					Eigen::Vector3f queryPoint = worldTcamera*Eigen::Vector3f(x, y, z);

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
	for (octomap::OcTree::iterator it = tree->begin(tree->getTreeDepth()),
		     end = tree->end(); it != end; ++it)
	{
		if (tree->isNodeOccupied(*it))
		{
			unsigned idx = it.getDepth();

			pts.emplace_back(it.getCoordinate());
		}
	}
	return pts;
}

std::pair<octomap::point3d_collection, std::shared_ptr<octomap::OcTree>> getOcclusionsInFOV(
	const octomap::OcTree* octree,
	const image_geometry::PinholeCameraModel& cameraModel,
	const Eigen::Affine3d& cameraTtable,
	const Eigen::Vector3f& minExtent,
	const Eigen::Vector3f& maxExtent)
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
					if (!octree->search(ix, iy, iz))
					{
						//This cell is unknown
						octomap::point3d coord = octree->keyToCoord(octree->coordToKey(ix, iy, iz, d), d);

						octomap::point3d ray = coord-cameraOrigin;
						octomap::point3d collision;
						bool collided = octree->castRay(cameraOrigin, ray, collision, false);

						if (collided && ray.norm_sq() > (collision-cameraOrigin).norm_sq())
						{
							#pragma omp critical
							{
								occluded_pts.push_back(coord);
								occlusionTree->setNodeValue(coord, std::numeric_limits<float>::infinity(), true);
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
		if ((!base_color && tree->isNodeOccupied(*it)) || (base_color && it->getOccupancy() > 0.25))
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
				colormap(igl::parula_cm, (float)((it->getOccupancy()-0.5)*2.0), color.r, color.g, color.b);
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

		if (occupiedNodesVis.markers[i].points.size()>0)
			occupiedNodesVis.markers[i].action = visualization_msgs::Marker::ADD;
		else
			occupiedNodesVis.markers[i].action = visualization_msgs::Marker::DELETE;
	}

		return occupiedNodesVis;
}