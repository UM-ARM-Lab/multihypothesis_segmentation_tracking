//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/ParticleFilter.h"

namespace mps
{

ParticleFilter::ParticleFilter(int n) : numParticles(n)
{
	particles.resize(n);
}

Particle ParticleFilter::applyActionModel(const Particle& inputParticle)
{
	Particle outputParticle;

	return outputParticle;
}


}