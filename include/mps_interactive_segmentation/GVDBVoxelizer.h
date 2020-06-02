//
// Created by pricear on 2020-04-24.
//

#ifndef MPS_GVDBVOXELIZER_H
#define MPS_GVDBVOXELIZER_H

#include "mps_interactive_segmentation/SceneVoxelizer.h"
#include <mps_voxels/VoxelRegion.h>

#include <geometric_shapes/shapes.h>

#include <memory>

namespace nvdb
{
class VolumeGVDB;
}

namespace mps
{

struct GLContext;

class GVDBVoxelizer : public SceneVoxelizer
{
public:
	std::unique_ptr<nvdb::VolumeGVDB> gvdb;
	std::unique_ptr<GLContext> context;

	GVDBVoxelizer(const mps::VoxelRegion& region, const std::vector<const shapes::Mesh*>& meshes);
	~GVDBVoxelizer();

	mps::VoxelRegion::VertexLabels voxelize(const mps::VoxelRegion& region, const std::vector<Eigen::Isometry3d>& poses) override;
};

}

#endif //MPS_GVDBVOXELIZER_H
