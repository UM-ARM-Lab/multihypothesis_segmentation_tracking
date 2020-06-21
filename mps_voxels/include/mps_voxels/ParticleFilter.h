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

class Scenario;
struct RigidTF;
class ObjectActionModel;

class ParticleFilter
{
public:
	using MotionModel = std::map<ObjectIndex, RigidTF>; // TODO: This should be probabilistic
	using MeasurementSensorData = Scene;
	using ActionSensorData = SensorHistoryBuffer;

	// TODO: Classes for storing intermediate results

	ParticleFilter(std::shared_ptr<const Scenario> scenario_, const double& res, const Eigen::Vector3d& rmin, const Eigen::Vector3d& rmax, int n=10);

	std::shared_ptr<const Scenario> scenario;

	std::shared_ptr<VoxelRegion> voxelRegion;

	int numParticles;
	std::vector<Particle> particles;

	int generation = 0;

	bool initializeParticles(const std::shared_ptr<const MeasurementSensorData>& data);
	std::vector<Particle> createParticlesFromSegmentation(const std::shared_ptr<const MeasurementSensorData>& data, const int n) const;

	std::pair<ParticleIndex, MotionModel>
	computeActionModel(
		const Particle& inputParticle,
		const ActionSensorData& buffer,
		std::unique_ptr<Tracker>& sparseTracker,
		std::unique_ptr<DenseTracker>& denseTracker) const;

	Particle applyActionModel(
		const Particle& inputParticle,
		const ParticleFilter::MotionModel& action) const;

	void computeAndApplyActionModel(
		const ActionSensorData& buffer,
		std::unique_ptr<Tracker>& sparseTracker,
		std::unique_ptr<DenseTracker>& denseTracker);

	void applyMeasurementModel(const std::shared_ptr<const Scene>& newScene);

	void resample(std::default_random_engine& rng);
};

}
#endif //SRC_PARTICLEFILTER_H
