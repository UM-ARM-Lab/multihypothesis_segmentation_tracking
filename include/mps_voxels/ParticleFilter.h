//
// Created by kunhuang on 2/12/20.
//

#ifndef SRC_PARTICLEFILTER_H
#define SRC_PARTICLEFILTER_H

#include "mps_voxels/VoxelSegmentation.h"
#include "mps_voxels/Scene.h"

namespace mps
{

class ParticleFilter
{
public:
	explicit ParticleFilter(int n=10);

	int numParticles;
	std::vector<VoxelSegmentation> particles;
};

}
#endif //SRC_PARTICLEFILTER_H
