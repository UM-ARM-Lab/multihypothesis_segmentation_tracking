//
// Created by arprice on 7/24/18.
//

#ifndef PROJECT_OCTREE_UTILS_H
#define PROJECT_OCTREE_UTILS_H

#include <octomap/octomap.h>
#include <ros/ros.h>
#include <memory>

class OctreeRetriever
{
public:
	OctreeRetriever(ros::NodeHandle& nh);
	ros::ServiceClient mapClient;
	std::pair<std::shared_ptr<octomap::OcTree>, std::string> getOctree();
};

#endif //PROJECT_OCTREE_UTILS_H
