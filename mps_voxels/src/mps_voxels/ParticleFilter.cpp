//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/ParticleFilter.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/SegmentationTreeSampler.h"
#include "mps_voxels/image_output.h"
#include "mps_voxels/JaccardMatch.h"
#include "mps_voxels/OccupancyData.h"

#include "mps_voxels/hacks.h"

#include <tf_conversions/tf_eigen.h>

#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_cv_mat.h"
#include "mps_voxels/logging/log_siammask.h"
#include <boost/filesystem.hpp>

namespace mps
{
const bool shouldLogSiamMask = true;

ParticleFilter::ParticleFilter(std::shared_ptr<const Scenario> scenario_, const double& res, const Eigen::Vector3d& rmin, const Eigen::Vector3d& rmax, int n, int mix)
	: scenario(std::move(scenario_)), numParticles(n), numNewSceneParticle(mix)
{
	voxelRegion = std::make_shared<VoxelRegion>(res, rmin, rmax, scenario->worldFrame);
}

bool
ParticleFilter::initializeParticles(
	const std::shared_ptr<const MeasurementSensorData>& scene)
{
	assert(particles.empty());

	particles = createParticlesFromSegmentation(scene, numParticles);

	return static_cast<int>(particles.size()) == numParticles;
}

std::vector<Particle>
ParticleFilter::createParticlesFromSegmentation(const std::shared_ptr<const MeasurementSensorData>& scene, const int n) const
{
	std::vector<Particle> newParticles;
	newParticles.reserve(n);

	SegmentationTreeSampler treeSampler(scene->segInfo);
	std::vector<std::pair<double, SegmentationCut>> segmentationSamples = treeSampler.sample(scenario->rng(), n, true);

	// Currently this is only local, but may need to move up to the class level
	std::map<ParticleIndex, std::shared_ptr<SegmentationInfo>> particleToInitialSegmentation;

	for (int p = 0; p < n; ++p)
	{
		// Generate a particle corresponding to this segmentation
		Particle particle;
		particle.particle.id = p;

		const auto& sample = segmentationSamples[p];

		particle.state = std::make_shared<OccupancyData>(voxelRegion);
		particle.weight = sample.first;

		const auto segInfo = std::make_shared<SegmentationInfo>(sample.second.segmentation);
		particleToInitialSegmentation.emplace(p, segInfo);
		// Visualization:
		IMSHOW("segmentation", colorByLabel(segInfo->objectness_segmentation->image));

		bool execSegmentation = SceneProcessor::performSegmentation(*scene, segInfo->objectness_segmentation->image, *particle.state);
		if (!execSegmentation)
		{
			std::cerr << "Particle " << particle.particle << " failed to segment." << std::endl;
			return {};
		}

		bool getCompletion = SceneProcessor::buildObjects(*scene, *particle.state);
		if (!getCompletion)
		{
			return {};
		}

		particle.state->vertexState = voxelRegion->objectsToSubRegionVoxelLabel(particle.state->objects, scene->minExtent.head<3>());
		particle.state->uniqueObjectLabels = getUniqueObjectLabels(particle.state->vertexState);

		auto uniqueImageLabels = unique(segInfo->objectness_segmentation->image);

		newParticles.push_back(particle);
	}

	return newParticles;
}

std::pair<ParticleIndex, ParticleFilter::MotionModel>
ParticleFilter::computeActionModel(
	const Particle& inputParticle,
	const ActionSensorData& buffer,
	std::unique_ptr<Tracker>& sparseTracker,
	std::unique_ptr<DenseTracker>& denseTracker) const
{
	ensureGPUMemory();

	// Return type
	MotionModel labelToMotionLookup;

	// Handle empty buffer
	if (buffer.rgb.empty())
	{
		const auto uniqueVoxelLabels = getUniqueObjectLabels(inputParticle.state->vertexState);
		for (const ObjectIndex& label : uniqueVoxelLabels)
		{
			RigidTF ident;
			ident.tf = mps::Pose::Identity();
			labelToMotionLookup.emplace(label.id, ident);
		}

		return {inputParticle.particle, labelToMotionLookup};
	}

	moveit::Pose worldTcamera;
	const auto& cameraModel = buffer.cameraModel;
	{
		const ros::Time queryTime = ros::Time(0); // buffer.rgb.begin()->first;
		const ros::Duration timeout = ros::Duration(5.0);
		std::string tfError;
		bool canTransform = buffer.tfs->canTransform(scenario->worldFrame, cameraModel.tfFrame(), queryTime, &tfError);

		if (!canTransform) // ros::Duration(5.0)
		{
			ROS_ERROR_STREAM("Failed to look up transform between '" << scenario->worldFrame << "' and '"
			                                                        << cameraModel.tfFrame() << "' with error '"
			                                                        << tfError << "'.");
			throw std::runtime_error("Sadness.");
		}

		tf::StampedTransform cameraFrameInTableCoordinates;
		const auto temp = buffer.tfs->lookupTransform(cameraModel.tfFrame(), scenario->worldFrame, queryTime);
		tf::transformStampedMsgToTF(temp, cameraFrameInTableCoordinates);
		tf::transformTFToEigen(cameraFrameInTableCoordinates.inverse(), worldTcamera);
	}

	// Image region of objects currently
	const cv::Rect objectsROI = occupancyToROI(*inputParticle.state, cameraModel, worldTcamera);
	cv::Mat segParticle = rayCastOccupancy(*inputParticle.state, cameraModel, worldTcamera, objectsROI);

	// Image region where objects could be after moving
//	const cv::Rect workspaceROI = workspaceToROI(*inputParticle.state->voxelRegion, cameraModel, worldTcamera);

//	const cv::Mat& segParticle = inputParticle.state->segInfo->objectness_segmentation->image;
//	const cv::Rect& roi = inputParticle.state->segInfo->roi;

	using LabelT = uint16_t;
	std::map<LabelT, mps_msgs::AABBox2d> labelToBBoxLookup = getBBox(segParticle, objectsROI, 5);
	std::cerr << "number of bounding boxes in segParticle: " << labelToBBoxLookup.size() << std::endl;

	SiamMaskData siammasks;
	for (auto& pair : labelToBBoxLookup)
	{
		const auto label = pair.first;
		std::shared_ptr<const ObjectActionModel> oam = estimateMotion(scenario, buffer, segParticle, pair.first, pair.second, sparseTracker, denseTracker, 1);
		if (oam)
		{
			std::string temp = "";
			if (shouldLogSiamMask && !denseTracker->isHistoryTracker(temp)) { siammasks.emplace(label, oam->masks); }
			labelToMotionLookup.emplace(label, oam->actionSamples[0]);
		}
		else
		{
			ROS_ERROR_STREAM("Failed to sample action for label " << pair.first << " !!!");
			//TODO: generate reasonable disturbance
			RigidTF randomSteadyTF;
			randomSteadyTF.tf = mps::Pose::Identity();
			labelToMotionLookup.emplace(label, randomSteadyTF);
		}
	}
	std::string temp = "";
	if (shouldLogSiamMask && !denseTracker->isHistoryTracker(temp))
	{
		std::string trackingFilename =
			scenario->experiment->experiment_dir + "/"
			+ "dense_track_"
			+ std::to_string(generation) + "_"
			+ std::to_string(inputParticle.particle.id) + ".bag";
		if (!temp.empty()) trackingFilename = temp + "dense_track_" + std::to_string(generation) + "_" + std::to_string(inputParticle.particle.id) + ".bag";
		DataLog logger(trackingFilename);
		logger.log<SiamMaskData>("SiamMaskData", siammasks);
		ROS_INFO_STREAM("Logged siammasks");
	}

	// Validation code
	const auto uniqueVoxelLabels = getUniqueObjectLabels(inputParticle.state->vertexState);
	const auto uniqueImageLabels = unique(segParticle);
	for (const ObjectIndex& label : uniqueVoxelLabels)
	{
		if (labelToMotionLookup.find(label) == labelToMotionLookup.end())
		{
//			std::cerr << "ERROR!!! No motion generated for object label '" << label << "'." << std::endl;
			RigidTF ident;
			ident.tf = mps::Pose::Identity();
			labelToMotionLookup.emplace(label.id, ident);
//			MPS_ASSERT(labelToMotionLookup.find(label) != labelToMotionLookup.end());
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
	outputParticle.state = std::make_shared<OccupancyData>(inputParticle.state->voxelRegion, outputState);
	outputParticle.particle = inputParticle.particle;

	return outputParticle;
}


void ParticleFilter::computeAndApplyActionModel(
	const ParticleFilter::ActionSensorData& buffer,
	std::unique_ptr<Tracker>& sparseTracker,
	std::unique_ptr<DenseTracker>& denseTracker)
{
	ros::Time finalTime = buffer.rgb.rbegin()->first;
	for (size_t p = 0; p < this->particles.size(); ++p)
	{
		std::string trackingFilename = "";
		if (denseTracker->isHistoryTracker(trackingFilename))
		{
			if (generation == 0)
			{
				trackingFilename[trackingFilename.size()-5] = '0' + p;
				denseTracker = std::make_unique<HistoryTracker>(trackingFilename);
			}
			else denseTracker = std::make_unique<SiamTracker>();
		}
		auto motion = computeActionModel(particles[p], buffer, sparseTracker, denseTracker);
		auto newParticle = applyActionModel(particles[p], motion.second);
		newParticle.particle.id = p;
		newParticle.time = finalTime;
		particles[p] = newParticle;
	}
	++generation;
}

void ParticleFilter::refine(const std::shared_ptr<const MeasurementSensorData>& newScene)
{
	/////////////////////////////////////////////
	//// Generate optimal particle for newScene
	/////////////////////////////////////////////
	newSceneParticle.state = std::make_shared<OccupancyData>(voxelRegion);

	const auto segInfo = newScene->segInfo;

	bool execSegmentation = SceneProcessor::performSegmentation(*newScene, segInfo->objectness_segmentation->image, *newSceneParticle.state);
	if (!execSegmentation)
	{
		std::cerr << "New Optimal Particle failed to segment." << std::endl;
	}

	bool getCompletion = SceneProcessor::buildObjects(*newScene, *newSceneParticle.state);
	if (!getCompletion)
	{
		std::cerr << "New Optimal Particle failed to shape complete." << std::endl;
	}

	newSceneParticle.state->vertexState = voxelRegion->objectsToSubRegionVoxelLabel(newSceneParticle.state->objects, newScene->minExtent.head<3>());
	newSceneParticle.state->uniqueObjectLabels = getUniqueObjectLabels(newSceneParticle.state->vertexState);


	for (auto & particle : particles)
	{
		assert(particle.state);
		/////////////////////////////////////////////
		//// Free space refinement
		/////////////////////////////////////////////
		refineParticleFreeSpace(particle, newScene->sceneOctree, 0.03);
	}
}

void ParticleFilter::applyMeasurementModel(const std::shared_ptr<const MeasurementSensorData>& newScene)
{
	const size_t nSamples = 6;
	SegmentationTreeSampler treeSampler(newScene->segInfo);
	std::vector<std::pair<double, SegmentationCut>> segmentationSamples = treeSampler.sample(scenario->rng(), nSamples, true);

	assert(newScene->roi.x >= 0);
	assert(newScene->roi.y >= 0);
	assert(newScene->roi.width > 0);
	assert(newScene->roi.height > 0);

	// Compute fitness of all particles w.r.t. this segmentation
	for (auto & particle : particles)
	{
		assert(particle.state);

		cv::Mat segParticle = rayCastOccupancy(*particle.state, newScene->cameraModel, newScene->worldTcamera, newScene->roi);
		if (segParticle.size() == cv::Size(newScene->cameraModel.cameraInfo().width, newScene->cameraModel.cameraInfo().height))
		{
			cv::Mat cropped = segParticle(newScene->roi);
			cropped.copyTo(segParticle);
		}
		MPS_ASSERT(segParticle.size() == newScene->roi.size());
		// TODO: cache somewhere?

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
//			double matchScore = J.symmetricCover();
			double matchScore = J.match.first;
			if (matchScore > bestMatchScore)
			{
				bestMatchScore = matchScore;
				segWeight = segmentationSamples[s].first;
			}
		}
		std::cerr << "particle " << particle.particle.id << " best match score " << bestMatchScore << std::endl;
		std::cerr << "particle " << particle.particle.id << " relative segmentation weight " << segWeight << std::endl;
		particle.weight += segWeight; // = or += ?
	}
}

void ParticleFilter::resample(std::mt19937& rng)
{
	// TODO: introduce newSensorParticle to particleFilter, resampling for only (numParticle - newSensorParticle) times
	std::vector<double> weightBar;

	for (auto &p : particles)
	{
		weightBar.push_back(p.weight);
	}
	std::discrete_distribution<> distribution(weightBar.begin(), weightBar.end());
	std::cout << "Probabilities: ";
	for (double x:distribution.probabilities()) std::cout << x << " ";
	std::cout << std::endl;

	std::vector<Particle> resampledParticles;
	for (int i = 0; i < numParticles; ++i)
	{
		int index = distribution(rng);
		resampledParticles.push_back(particles[index]);
	}
	/// Should be deep copy!
	for (int i = 0; i < numParticles; ++i)
	{
		particles[i] = resampledParticles[i];
	}
}

}