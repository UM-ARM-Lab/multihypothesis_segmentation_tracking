//
// Created by arprice on 9/7/18.
//

#ifndef MPS_PLANNING_STATE_H
#define MPS_PLANNING_STATE_H

#include "mps_voxels/MotionModel.h"
#include "mps_voxels/ObjectIndex.h"
#include "mps_voxels/moveit_pose_type.h"

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <vector>
#include <map>
#include <memory>

namespace mps
{

class State
{
public:
	using Pose = moveit::Pose;
	using Poses = std::map<ObjectIndex, Pose, std::less<>, Eigen::aligned_allocator<Pose>>;
	Poses poses;
};


class StateSpace
{
public:
	std::vector<std::shared_ptr<MotionModel>> motion_models;
	std::map<std::string, ObjectIndex> model_name_lookup;
};

}

#endif // MPS_PLANNING_STATE_H
