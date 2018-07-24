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