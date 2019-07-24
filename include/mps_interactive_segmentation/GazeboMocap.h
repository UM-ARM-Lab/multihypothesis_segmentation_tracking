//
// Created by kunhuang on 7/23/19.
//

#ifndef SRC_GAZEBOMOCAP_H
#define SRC_GAZEBOMOCAP_H

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <Eigen/Geometry>

namespace mps
{

Eigen::Isometry3d randomTransform(const double tScale = 1.0);

struct GazeboFrame
{
	std::string modelName;
	std::string linkName;
	std::string gazeboName() const { return modelName; }
	std::string tfName() const { return modelName; }
	//std::string gazeboName() const { return modelName + "::" + linkName; }
	//std::string tfName() const { return modelName + "_" + linkName; }
};

bool operator<(const GazeboFrame& lhs, const GazeboFrame& rhs)
{
	if (lhs.modelName < rhs.modelName)
	{
		return true;
	}
	return lhs.linkName < rhs.linkName;
}

class GazeboMocap
{
public:
	const std::string mocap_frame_id = "mocap";

	std::unique_ptr<ros::NodeHandle> nh;
	std::unique_ptr<tf::TransformBroadcaster> tb;
	ros::ServiceClient worldStateClient;
	ros::ServiceClient modelStateClient;

	tf::StampedTransform gazeboTmocap;
	std::map<const GazeboFrame, tf::StampedTransform> linkPoses;
	std::map<const GazeboFrame, tf::StampedTransform> markerOffsets;

	GazeboMocap();

	bool getTransforms();
	void sendTransforms(bool sendBaseFrame = false);
	std::vector<std::string> getModelNames();
};

}

#endif //SRC_GAZEBOMOCAP_H
