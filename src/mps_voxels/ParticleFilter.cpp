//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/ParticleFilter.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/SegmentationTreeSampler.h"
#include "mps_voxels/image_output.h"
#include "mps_voxels/JaccardMatch.h"

namespace mps
{

ParticleFilter::ParticleFilter(std::shared_ptr<const Scenario> scenario_, const VoxelRegion::vertex_descriptor& dims, const double& res, const Eigen::Vector3d& rmin, const Eigen::Vector3d& rmax, int n)
	: scenario(std::move(scenario_)), numParticles(n)
{
	voxelRegion = std::make_shared<VoxelRegion>(dims, res, rmin, rmax);;
//	particles.resize(n);
//	for (int i=0; i<n; ++i)
//	{
//		particles[i].state = std::make_shared<Particle::ParticleData>(voxelRegion);
//	}
}

bool
ParticleFilter::initializeParticles(
	const std::shared_ptr<const MeasurementSensorData>& scene)
{
	assert(particles.empty());
	particles.reserve(numParticles);

	SegmentationTreeSampler treeSampler(scene->segInfo);
	std::vector<std::pair<double, SegmentationCut>> segmentationSamples = treeSampler.sample(scenario->rng(), numParticles, true);

	// TODO: This should probably be wrapped into its own thing
	for (int p = 0; p < numParticles; ++p)
	{
		// Generate a particle corresponding to this segmentation
		Particle particle;
		particle.particle.id = p;

		const auto& sample = segmentationSamples[p];

		particle.state = std::make_shared<OccupancyData>(voxelRegion);
		particle.state->segInfo = std::make_shared<SegmentationInfo>(sample.second.segmentation);
		particle.weight = sample.first;
		bool execSegmentation = SceneProcessor::performSegmentation(*scene, particle.state->segInfo, *particle.state);
		if (!execSegmentation)
		{
			std::cerr << "Particle " << particle.particle << " failed to segment." << std::endl;
			return false;
		}

		bool getCompletion = SceneProcessor::buildObjects(*scene, *particle.state);
		if (!getCompletion)
		{
			return false;
		}

		particle.state->vertexState = voxelRegion->objectsToSubRegionVoxelLabel(particle.state->objects, scene->minExtent.head<3>().cast<double>());
		particle.state->uniqueObjectLabels = getUniqueObjectLabels(particle.state->vertexState);
		particle.state->parentScene = scene;

		auto uniqueImageLabels = unique(particle.state->segInfo->objectness_segmentation->image);


		particles.push_back(particle);
	}

	return true;
}

std::pair<ParticleIndex, ParticleFilter::MotionModel>
ParticleFilter::computeActionModel(
	const Particle& inputParticle,
	const ActionSensorData& buffer,
	std::unique_ptr<Tracker>& sparseTracker,
	std::unique_ptr<DenseTracker>& denseTracker) const
{
	const cv::Mat& segParticle = inputParticle.state->segInfo->objectness_segmentation->image;
	const cv::Rect& roi = inputParticle.state->segInfo->roi;
	// TODO: Use faster version
	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup = getBBox(segParticle, roi, 5);
	std::cerr << "number of bounding boxes in segParticle: " << labelToBBoxLookup.size() << std::endl;

	std::unique_ptr<ObjectActionModel> oam = std::make_unique<ObjectActionModel>(scenario, 1);
	std::map<ObjectIndex, RigidTF> labelToMotionLookup;
	for (auto& pair : labelToBBoxLookup)
	{
		bool sampleActionSuccess = oam->sampleAction(buffer, segParticle, roi, sparseTracker, denseTracker, pair.first, pair.second);
		if (sampleActionSuccess)
		{
			labelToMotionLookup.emplace(pair.first, oam->actionSamples[0]);
		}
		else
		{
			ROS_ERROR_STREAM("Failed to sample action for label " << pair.first << " !!!");
			//TODO: generate reasonable disturbance
			RigidTF randomSteadyTF;
			randomSteadyTF.tf = mps::Pose::Identity();
			labelToMotionLookup.emplace(pair.first, randomSteadyTF);
		}
	}

	// Validation code
	const auto uniqueVoxelLabels = getUniqueObjectLabels(inputParticle.state->vertexState);
	const auto uniqueImageLabels = unique(segParticle);
	for (const ObjectIndex& label : uniqueVoxelLabels)
	{
		if (labelToMotionLookup.find(label) == labelToMotionLookup.end())
		{
//			std::cerr << "ERROR!!! No motion generated for object label '" << label << "'." << std::endl;
//			RigidTF ident;
//			ident.tf = mps::Pose::Identity();
//			labelToMotionLookup.emplace(label.id, ident);
			MPS_ASSERT(labelToMotionLookup.find(label) != labelToMotionLookup.end());
		}
	}

	return {inputParticle.particle, labelToMotionLookup};
}

Particle ParticleFilter::applyActionModel(
	const Particle& inputParticle,
	const ParticleFilter::MotionModel& action) const
{
	const VoxelRegion& region = *inputParticle.state->voxelRegion;
	assert(inputParticle.state->vertexState.size() == region.num_vertices());
	VoxelRegion::VertexLabels outputState(region.num_vertices(), VoxelRegion::FREE_SPACE);

//#pragma omp parallel for
	for (size_t i = 0; i < inputParticle.state->vertexState.size(); ++i)
	{
		if (inputParticle.state->vertexState[i] != VoxelRegion::FREE_SPACE)
		{
			const auto& T = action.at(ObjectIndex(inputParticle.state->vertexState[i])).tf;
			VoxelRegion::vertex_descriptor vd = region.vertex_at(i);
			Eigen::Vector3d originalCoord = vertexDescpToCoord(region.resolution, region.regionMin, vd);
			Eigen::Vector3d newCoord = T * originalCoord;

			if (!region.isInRegion(newCoord)) continue;

			VoxelRegion::vertex_descriptor newVD = coordToVertexDesc(region.resolution, region.regionMin, newCoord);
			auto index = region.index_of(newVD);
			outputState[index] = inputParticle.state->vertexState[i];
		}
	}

	Particle outputParticle;
	outputParticle.state = std::make_shared<OccupancyData>(
		inputParticle.state->parentScene.lock(), inputParticle.state->voxelRegion, outputState);

	return outputParticle;
}


void ParticleFilter::computeAndApplyActionModel(
	const ParticleFilter::ActionSensorData& buffer,
	std::unique_ptr<Tracker>& sparseTracker,
	std::unique_ptr<DenseTracker>& denseTracker)
{
	for (size_t p = 0; p < this->particles.size(); ++p)
	{
		auto motion = computeActionModel(particles[p], buffer, sparseTracker, denseTracker);
		auto newParticle = applyActionModel(particles[p], motion.second);
		particles[p] = newParticle;
	}
}

void ParticleFilter::applyMeasurementModel(const std::shared_ptr<const Scene>& newScene)
{
	const size_t nSamples = 3;
	SegmentationTreeSampler treeSampler(newScene->segInfo);
	std::vector<std::pair<double, SegmentationCut>> segmentationSamples = treeSampler.sample(scenario->rng(), nSamples, true);

	// Compute fitness of all particles w.r.t. this segmentation
	for (auto & particle : particles)
	{
		particle.state->parentScene = newScene; // Update the scene pointer
		// Ray-cast particle only in ROI
		cv::Mat segParticle(particle.state->segInfo->objectness_segmentation->image, newScene->roi);

		IMSHOW("segmentation", colorByLabel(segParticle));
		WAIT_KEY(0);

		double bestMatchScore = -std::numeric_limits<double>::infinity();
		double segWeight = -std::numeric_limits<double>::infinity();

		for (size_t s = 0; s < nSamples; ++s)
		{
			const auto& segSample = segmentationSamples[s].second.segmentation.objectness_segmentation->image;
			MPS_ASSERT(newScene->roi.size() == segSample.size());
			MPS_ASSERT(segParticle.size() == segSample.size());

			mps::JaccardMatch J(segParticle, segSample);
			double matchScore = J.symmetricCover();
			if (matchScore > bestMatchScore)
			{
				bestMatchScore = matchScore;
				segWeight = segmentationSamples[s].first;
			}
		}

		particle.weight += segWeight;
	}
}


}