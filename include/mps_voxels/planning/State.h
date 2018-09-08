//
// Created by arprice on 9/7/18.
//

#ifndef MPS_PLANNING_STATE_H
#define MPS_PLANNING_STATE_H

#include "mps_voxels/MotionModel.h"

#include <Eigen/StdVector>
#include <Eigen/Geometry>

#include <vector>
#include <map>
#include <memory>

class State
{
public:
	using Pose = Eigen::Affine3d;
	using Poses = std::vector<Pose, Eigen::aligned_allocator<Pose>>;
	Poses poses;
};


class StateSpace
{
public:
	std::vector<std::shared_ptr<MotionModel>> motion_models;
	std::map<std::string, int> model_name_lookup;
};

#endif // MPS_PLANNING_STATE_H
