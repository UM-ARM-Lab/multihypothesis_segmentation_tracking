//
// Created by arprice on 7/24/18.
//

#include "mps_voxels/octree_utils.h"

#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

namespace om = octomap;

OctreeRetriever::OctreeRetriever(ros::NodeHandle& nh)
{
	mapClient = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
	if (!mapClient.waitForExistence(ros::Duration(10)))
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

visualization_msgs::MarkerArray visualizeOctree(std::shared_ptr<octomap::OcTree>& tree, const std::string& globalFrame)
{
	visualization_msgs::MarkerArray occupiedNodesVis;
	occupiedNodesVis.markers.resize(tree->getTreeDepth()+1);

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

			// TODO: Colors
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

		return occupiedNodesVis;
}