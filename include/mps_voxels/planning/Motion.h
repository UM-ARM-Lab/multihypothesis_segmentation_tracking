//
// Created by arprice on 9/7/18.
//

#ifndef MPS_PLANNING_MOTION_H
#define MPS_PLANNING_MOTION_H

#include "mps_voxels/planning/State.h"
#include "mps_voxels/planning/Action.h"

#include <memory>

namespace mps
{

struct Motion
{
	std::shared_ptr<State> state;
	std::shared_ptr<Action> action;
	std::shared_ptr<Motion> parent;

	std::vector<ObjectIndex> targets;
};

}

#endif //MPS_PLANNING_MOTION_H
