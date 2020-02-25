//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/Particle.h"
#include "mps_voxels/OccupancyData.h"
#include "mps_voxels/moveit_pose_type.h"

namespace mps
{


void refineParticleFreeSpace(Particle& particle, const octomap::OcTree* sceneOctree)
{
	Eigen::Vector3d roiMin = particle.state->voxelRegion->regionMin;
	for (int i = 0; i < (int)particle.state->voxelRegion->num_vertices(); ++i)
	{
		if (particle.state->vertexState[i] >= 0)
		{
			VoxelRegion::vertex_descriptor vd = particle.state->voxelRegion->vertex_at(i);
			Eigen::Vector3d coord = particle.state->voxelRegion->coordinate_of(vd);
			octomap::OcTreeNode* node = sceneOctree->search(coord.x(), coord.y(), coord.z());

			if (node) /// coord is inside sceneOctree; if it is not: then it's unseen
			{
				if (node->getOccupancy() <= sceneOctree->getOccupancyThres())
				{
					particle.state->vertexState[i] = -1; /// set to empty
				}
			}
		}
	}
}

}