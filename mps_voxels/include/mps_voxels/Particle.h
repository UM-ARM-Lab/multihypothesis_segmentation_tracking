/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef MPS_PARTICLE_H
#define MPS_PARTICLE_H

#include "mps_voxels/Indexes.h"
#include "mps_voxels/VoxelRegion.h"
#include "mps_voxels/moveit_pose_type.h"

#include <opencv2/core.hpp>

namespace image_geometry
{
class PinholeCameraModel;
}

namespace mps
{

struct OccupancyData;

struct Particle
{
	using ParticleData = OccupancyData;

	// Indexing properties of this particular particle
	TimeIndex time;
	SubproblemIndex problem;
//	ObjectIndex object;
	ParticleIndex particle;

	// The actual state data and its cached computations
	std::shared_ptr<ParticleData> state;

	// Our belief weight of this particle
	double weight = 0;
};

void refineParticleFreeSpace(Particle& particle, const octomap::OcTree* sceneOctree);

void refineNewOptimalParticleBased(Particle& particle, const Particle& newOptimalParticle);
}

#endif //MPS_PARTICLE_H
