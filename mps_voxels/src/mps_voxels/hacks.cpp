//
// Created by pricear on 2020-07-07.
//

#include "mps_voxels/hacks.h"

#include <cstdlib>

namespace mps
{

void logParameterServer()
{
	std::system("bash -c 'source /home/pricear/mps_ws/devel/setup.bash ; rosparam dump $(rosparam get /experiment/directory)/rosparam.yaml'");
}

void ensureGPUMemory()
{
	std::system("bash -c 'source /home/pricear/mps_ws/devel/setup.bash ; rosnode kill shape_completion_node'");
}

}
