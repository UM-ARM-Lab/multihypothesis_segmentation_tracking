//
// Created by kunhuang on 2/12/20.
//

#ifndef SRC_PARTICLEFILTER_H
#define SRC_PARTICLEFILTER_H

#include "mps_voxels/VoxelRegion.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/Particle.h"
#include "mps_voxels/ObjectActionModel.h"

namespace mps
{

class ParticleFilter
{
public:
	explicit ParticleFilter(int n=10);

	int numParticles;
	std::vector<Particle> particles;

	Particle applyActionModel(const Particle& inputParticle);
};

}
#endif //SRC_PARTICLEFILTER_H
