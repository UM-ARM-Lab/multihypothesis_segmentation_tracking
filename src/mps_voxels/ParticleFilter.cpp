//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/ParticleFilter.h"

namespace mps
{

ParticleFilter::ParticleFilter(VoxelRegion::vertex_descriptor dims, double res, Eigen::Vector3d rmin, Eigen::Vector3d rmax, int n)
	: voxRegion(dims, res, rmin, rmax), numParticles(n)
{
	particles.resize(n);
}

Particle ParticleFilter::applyActionModel(const Particle& /*inputParticle*/)
{
	Particle outputParticle;

	return outputParticle;
}


}