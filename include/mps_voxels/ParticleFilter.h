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
	ParticleFilter(VoxelRegion::vertex_descriptor dims, double res, Eigen::Vector3d rmin, Eigen::Vector3d rmax, int n=10);

	VoxelRegion voxRegion;

	int numParticles;
	std::vector<Particle> particles;

	Particle applyActionModel(const Particle& inputParticle, const image_geometry::PinholeCameraModel& cameraModel,
	                          const Eigen::Isometry3d& worldTcamera,
	                          SensorHistoryBuffer& buffer_out, SegmentationInfo& seg_out,
	                          std::unique_ptr<Tracker>& sparseTracker, std::unique_ptr<DenseTracker>& denseTracker,
	                          const int& segRes = 1);
};

}
#endif //SRC_PARTICLEFILTER_H
