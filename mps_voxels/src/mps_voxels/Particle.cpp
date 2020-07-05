//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/Particle.h"
#include "mps_voxels/OccupancyData.h"
#include "mps_voxels/moveit_pose_type.h"

namespace mps
{


std::pair<int, int> refineParticleFreeSpace(Particle& particle, const octomap::OcTree* sceneOctree, double table_height)
{
	int numVoxelEliminated = 0;
	int origTotalVoxels = 0;
	for (int i = 0; i < (int)particle.state->voxelRegion->num_vertices(); ++i)
	{
		if (particle.state->vertexState[i] != VoxelRegion::FREE_SPACE)
		{
			++origTotalVoxels;
			VoxelRegion::vertex_descriptor vd = particle.state->voxelRegion->vertex_at(i);
			Eigen::Vector3d coord = particle.state->voxelRegion->coordinate_of(vd);
			if (coord.z() < table_height)
			{
				particle.state->vertexState[i] = VoxelRegion::FREE_SPACE; /// eliminate voxels below table surface
				++numVoxelEliminated;
			}

			octomap::OcTreeNode* node = sceneOctree->search(coord.x(), coord.y(), coord.z());

			if (node) /// coord is inside sceneOctree; if it is not: then it's unseen
			{
				if (node->getOccupancy() <= sceneOctree->getOccupancyThres())
				{
					particle.state->vertexState[i] = VoxelRegion::FREE_SPACE; /// set to empty
					++numVoxelEliminated;
				}
			}
		}
	}
	return std::make_pair(numVoxelEliminated, origTotalVoxels);
}

/*
void refineNewOptimalParticleBased(Particle& particle, const Particle& newOptimalParticle)
{
}
*/

Particle filteringParticle(const Particle& inputParticle, bool& isconverge)
{
	//// TODO: set a threshold for changing current voxel; try different range
	isconverge = true;
	const VoxelRegion& region = *inputParticle.state->voxelRegion;
	assert(inputParticle.state->vertexState.size() == region.num_vertices());
	VoxelRegion::VertexLabels outputState(region.num_vertices(), VoxelRegion::FREE_SPACE);

//#pragma omp parallel for
	for (size_t i = 0; i < inputParticle.state->vertexState.size(); ++i)
	{
		if (inputParticle.state->vertexState[i] != VoxelRegion::FREE_SPACE)
		{
			VoxelRegion::vertex_descriptor vd = region.vertex_at(i);
			int range = 2;
			std::vector<size_t> neighbours = getNeighbourIndices(region, vd, range);
			std::map<int, int> neighbourLabels;
			for (auto& n : neighbours)
			{
				neighbourLabels[inputParticle.state->vertexState[n]]++;
			}
			int maxOccur = neighbourLabels[inputParticle.state->vertexState[i]] + 3; //// + 3?
			int maxRepeatingLabel = inputParticle.state->vertexState[i];
			for (auto& pair : neighbourLabels)
			{
				if (pair.first != VoxelRegion::FREE_SPACE && pair.second >= maxOccur)
				{
					maxOccur = pair.second; // great without this line, but sometimes the expanding direction is wrong
					maxRepeatingLabel = pair.first;
				}
			}

			if (neighbourLabels[VoxelRegion::FREE_SPACE] > pow(2*range+1, 3) - 2*range -1) { outputState[i] = VoxelRegion::FREE_SPACE; }
			else outputState[i] = maxRepeatingLabel;
//			else if (maxOccur > neighbourLabels.at(outputState[i])) outputState[i] = maxRepeatingLabel;
//			else outputState[i] = inputParticle.state->vertexState[i];

			if (outputState[i] != inputParticle.state->vertexState[i]) isconverge = false;
		}
	}

	Particle outputParticle;
	outputParticle.state = std::make_shared<OccupancyData>(inputParticle.state->voxelRegion, outputState);
	outputParticle.particle = inputParticle.particle;

	return outputParticle;
}

}