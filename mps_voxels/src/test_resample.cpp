//
// Created by kunhuang on 6/26/20.
//

#include <iostream>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <std_msgs/Float64.h>

#include "mps_voxels/ExperimentDir.h"
#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_sensor_history.h"
#include "mps_voxels/logging/log_scene.h"
#include "mps_voxels/logging/log_segmentation_info.h"

#include "mps_voxels/visualization/visualize_occupancy.h"
#include "mps_voxels/visualization/visualize_voxel_region.h"
#include <mps_voxels/visualization/dispersed_colormap.h>

#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/ParticleFilter.h"
#include "mps_voxels/util/package_paths.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/SegmentationTreeSampler.h"
#include "mps_voxels/Ultrametric.h"
#include "mps_voxels/image_output.h"
#include "mps_voxels/voxel_recombination.h"
#include "mps_voxels/JaccardMatch.h"
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/octree_utils.h"

using namespace mps;
using VoxelColormap = std::map<mps::VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>;

const std::string testDirName = "package://mps_test_data/";
const std::string expDirName = "2020-06-31/";
const std::string logDir = parsePackageURL(testDirName);
const int generation = 4;
const ExperimentDir checkpointDir;
const int numParticles = 5;
const bool isComputingWeight = true;

class ParticleFilterTestFixture
{
public:
	std::unique_ptr<ParticleFilter> particleFilter;
	std::unique_ptr<robot_model_loader::RobotModelLoader> mpLoader;
	robot_model::RobotModelPtr pModel;
	std::shared_ptr<Scenario> scenario;
	SensorHistoryBuffer motionData;

	void SetUp()
	{
		ros::NodeHandle nh, pnh("~");

		// Load robot and scenario configurations
		mpLoader = std::make_unique<robot_model_loader::RobotModelLoader>();
		pModel = mpLoader->getModel();
		scenario = scenarioFactory(nh, pnh, pModel);
		scenario->segmentationClient = std::make_shared<RGBDSegmenter>(nh);
		scenario->completionClient = std::make_shared<VoxelCompleter>(nh);

		Tracker::TrackingOptions opts;
		opts.directory = logDir + expDirName;

		double resolution = 0.010;
		pnh.getParam("resolution", resolution);
		particleFilter = std::make_unique<ParticleFilter>(scenario, resolution,
		                                                  scenario->minExtent.head<3>(),
		                                                  scenario->maxExtent.head<3>(), numParticles);

		/////////////////////////////////////////////
		//// Load buffer
		/////////////////////////////////////////////
		{
			DataLog loader(logDir + expDirName + "buffer_" + std::to_string(generation-1) + ".bag", {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("buffer");
			motionData = loader.load<SensorHistoryBuffer>("buffer");
			std::cerr << "Successfully loaded buffer." << std::endl;
		}
		std::cerr << "number of frames: " << motionData.rgb.size() << std::endl;
	}
};

std::pair<double, double>
applyMeasurementModel(const Particle& particle, const std::shared_ptr<const ParticleFilter::MeasurementSensorData>& newScene, std::mt19937& rng)
{
	const size_t nSamples = 10;
	SegmentationTreeSampler treeSampler(newScene->segInfo);
	std::vector<std::pair<double, SegmentationCut>> segmentationSamples = treeSampler.sample(rng, nSamples, true);

	assert(newScene->roi.x >= 0);
	assert(newScene->roi.y >= 0);
	assert(newScene->roi.width > 0);
	assert(newScene->roi.height > 0);

	assert(particle.state);

	cv::Mat segParticle = rayCastOccupancy(*particle.state, newScene->cameraModel, newScene->worldTcamera, newScene->roi);
	if (segParticle.size() == cv::Size(newScene->cameraModel.cameraInfo().width, newScene->cameraModel.cameraInfo().height))
	{
		cv::Mat cropped = segParticle(newScene->roi);
		cropped.copyTo(segParticle);
	}
	MPS_ASSERT(segParticle.size() == newScene->roi.size());

	double bestMatchScore = -std::numeric_limits<double>::infinity();
	double segWeight = -std::numeric_limits<double>::infinity();

	for (size_t s = 0; s < nSamples; ++s)
	{
		const auto& segSample = segmentationSamples[s].second.segmentation.objectness_segmentation->image;
		MPS_ASSERT(newScene->roi.size() == segSample.size());
		MPS_ASSERT(segParticle.size() == segSample.size());

		mps::JaccardMatch J(segParticle, segSample);
//		double matchScore = J.symmetricCover();
		double matchScore = J.match.first;
		if (matchScore > bestMatchScore)
		{
			bestMatchScore = matchScore;
			segWeight = segmentationSamples[s].first;
		}
	}
	std::cerr << "Best match score " << bestMatchScore << std::endl;
	std::cerr << "Relative segmentation weight " << segWeight << std::endl;
	return std::make_pair(segWeight, bestMatchScore);
}

std::vector<Particle> resampleLogWeights(const std::vector<Particle>& particles, int num, std::mt19937& rng)
{
	std::vector<double> weightBar;

	for (auto &p : particles)
	{
		weightBar.push_back(p.weight);
	}
	double minw = *std::min_element(weightBar.begin(), weightBar.end());
	for (auto& w : weightBar)
	{
		w+= minw;
		w = exp(w);
	}
	std::discrete_distribution<> distribution(weightBar.begin(), weightBar.end());

	std::vector<Particle> resampledParticles;
	for (int i = 0; i < num; ++i)
	{
		int index = distribution(rng);
		resampledParticles.push_back(particles[index]);
	}
	return resampledParticles;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_resample", ros::init_options::AnonymousName);
	ros::NodeHandle nh, pnh("~");

	ParticleFilterTestFixture fixture;
	fixture.SetUp();

	ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
	std::mt19937 rng;

	std::shared_ptr<Scene> newScene;
	if (isComputingWeight)
	{
		/////////////////////////////////////////////
		//// Initialize mapServer
		/////////////////////////////////////////////
		std::shared_ptr<Scenario> scenario = fixture.scenario;
		scenario->mapServer = std::make_shared<LocalOctreeServer>(0.01, "table_surface");
		octomap::point3d tMin(scenario->minExtent.x(), scenario->minExtent.y(), scenario->minExtent.z());
		octomap::point3d tMax(scenario->maxExtent.x(), scenario->maxExtent.y(), scenario->maxExtent.z());
		scenario->mapServer->m_octree->setBBXMin(tMin);
		scenario->mapServer->m_octree->setBBXMax(tMax);

		//// Compute new scene
		ros::Time finalTime = fixture.motionData.rgb.rbegin()->first;
		newScene = computeSceneFromSensorHistorian(scenario, fixture.motionData, finalTime, scenario->worldFrame);
	}


	std::vector<double> oldParticleQual;
	for (int i=0; i<numParticles; ++i)
	{
		Particle p;
		Eigen::Vector3d rmin(-1, -1, -1);
		Eigen::Vector3d rmax(1, 1, 1);
		std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);
		p.state = std::make_shared<Particle::ParticleData>(v);
		if (generation == 1)
		{
			DataLog loader(logDir + expDirName + checkpointDir.checkpoints[0] + "/particle_" +
			               std::to_string(generation-1)+ "_" + std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("particle");
			*p.state = loader.load<OccupancyData>("particle");
			std_msgs::Float64 weightMsg;
			loader.activeChannels.insert("weight");
			weightMsg = loader.load<std_msgs::Float64>("weight");

			oldParticleQual.push_back(weightMsg.data);
		}
		else
		{
			DataLog loader(logDir + expDirName + checkpointDir.checkpoints[2] + "/particle_" +
			               std::to_string(generation-1)+ "_" + std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("particle");
			*p.state = loader.load<OccupancyData>("particle");
			std_msgs::Float64 weightMsg;
			loader.activeChannels.insert("weight");
			weightMsg = loader.load<std_msgs::Float64>("weight");

			oldParticleQual.push_back(weightMsg.data);
		}
		p.particle.id = i;
		std::cerr << "Successfully loaded old particle_" << generation-1 << "_" << i <<" with quality " << p.weight << std::endl;
	}

	std::vector<double> newMeasurementParticleQual;
	for (int i=0; i<numParticles; ++i)
	{
		Particle p;
		Eigen::Vector3d rmin(-1, -1, -1);
		Eigen::Vector3d rmax(1, 1, 1);
		std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);
		p.state = std::make_shared<Particle::ParticleData>(v);
		DataLog loader(logDir + expDirName + checkpointDir.checkpoints[0] + "/particle_" +
		               std::to_string(generation)+ "_" + std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("particle");
		*p.state = loader.load<OccupancyData>("particle");
		p.particle.id = i;
		std_msgs::Float64 weightMsg;
		loader.activeChannels.insert("weight");
		weightMsg = loader.load<std_msgs::Float64>("weight");

		newMeasurementParticleQual.push_back(weightMsg.data);
		std::cerr << "Successfully loaded new measurement particle_" << generation << "_" << i <<" with quality " << p.weight << std::endl;
	}

	std::vector<double> refinedParticleQual;
	for (int i=0; i<numParticles; ++i)
	{
		Particle p;
		Eigen::Vector3d rmin(-1, -1, -1);
		Eigen::Vector3d rmax(1, 1, 1);
		std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);
		p.state = std::make_shared<Particle::ParticleData>(v);
		DataLog loader(logDir + expDirName + checkpointDir.checkpoints[4] + "/particle_" +
		               std::to_string(generation)+ "_" + std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("particle");
		*p.state = loader.load<OccupancyData>("particle");
		p.particle.id = i;
		std_msgs::Float64 weightMsg;
		loader.activeChannels.insert("weight");
		weightMsg = loader.load<std_msgs::Float64>("weight");

		refinedParticleQual.push_back(weightMsg.data);
		std::cerr << "Successfully loaded history particle_" << generation << "_" << i <<" with quality " << p.weight << std::endl;
	}

	Eigen::Vector3d rmin(-1, -1, -1);
	Eigen::Vector3d rmax(1, 1, 1);
	std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);
	std::vector<Particle> mixtureParticles;
	for (int newID=0; newID<numParticles; ++newID)
	{
		for (int oldID=0; oldID<numParticles; ++oldID)
		{
			for (int iter=0; iter<5; ++iter)
			{
				std::cout << "-------------------------------------------------------" << std::endl;
				Particle p;
				p.state = std::make_shared<Particle::ParticleData>(v);
				{
					DataLog loader(
						logDir + expDirName + checkpointDir.checkpoints[1] + "/particle_" + std::to_string(generation) +
						"_" + std::to_string(newID) + "_" + std::to_string(oldID) + "_" + std::to_string(iter) + ".bag",
						{}, rosbag::bagmode::Read);
					loader.activeChannels.insert("particle");
					*p.state = loader.load<OccupancyData>("particle");
//				    ROS_INFO_STREAM("Loaded mixed particle " << newID << " " << oldID);
					if (!isComputingWeight)
					{
						std_msgs::Float64 weightMsg;
						loader.activeChannels.insert("weight");
						weightMsg = loader.load<std_msgs::Float64>("weight");
						p.weight = weightMsg.data;
					}
				}

				std_msgs::Header header; header.frame_id = "table_surface"; header.stamp = ros::Time::now();
				auto pfMarkers = mps::visualize(*p.state, header, rng);
				visualPub.publish(pfMarkers);
				std::cerr << "mixed particle " << newID << " " << oldID << std::endl;
//				usleep(500000);

				std::cerr << oldParticleQual[oldID] << "    " << refinedParticleQual[oldID] << "    " << newMeasurementParticleQual[newID] << std::endl;

				p.weight = oldParticleQual[oldID] + newMeasurementParticleQual[newID] + 3*log(refinedParticleQual[oldID]);
				std::cerr << p.weight << std::endl;

				if (isComputingWeight)
				{
//					std::pair<double, double> measurementMatchness = applyMeasurementModel(p, newScene, rng);

					DataLog logger(logDir + expDirName + checkpointDir.checkpoints[1] + "/particle_" + std::to_string(generation) +
					               "_" + std::to_string(newID) + "_" + std::to_string(oldID) + "_" + std::to_string(iter) + ".bag",
					               {}, rosbag::bagmode::Append);
					std_msgs::Float64 weightMsg; weightMsg.data = p.weight;
					logger.activeChannels.insert("weight");
					logger.log<std_msgs::Float64>("weight", weightMsg);
/*
					std_msgs::Float64 matchnessMsg; matchnessMsg.data = measurementMatchness.first;
					logger.activeChannels.insert("relevantSeg");
					logger.log<std_msgs::Float64>("relevantSeg", matchnessMsg);
					matchnessMsg.data = measurementMatchness.second;
					logger.activeChannels.insert("matchness");
					logger.log<std_msgs::Float64>("matchness", matchnessMsg);
*/
				}
				mixtureParticles.push_back(p);
			}
		}
	}

	std::vector<Particle> resampledParticles = resampleLogWeights(mixtureParticles, numParticles, rng);
	for (size_t p = 0; p < resampledParticles.size(); ++p)
	{
		std_msgs::Header header; header.frame_id = "table_surface"; header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*resampledParticles[p].state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "resampled particle shown" << std::endl;
		usleep(500000);

		{
			DataLog logger(logDir + expDirName + checkpointDir.checkpoints[2] + "/particle_" + std::to_string(generation) + "_" +
			               std::to_string(p) + ".bag");
			logger.activeChannels.insert("particle");
			logger.log<OccupancyData>("particle", *resampledParticles[p].state);
			std_msgs::Float64 weightMsg; weightMsg.data = resampledParticles[p].weight;
			logger.activeChannels.insert("weight");
			logger.log<std_msgs::Float64>("weight", weightMsg);
			ROS_INFO_STREAM("Logged estimate particle " << p);
		}

	}

	return 0;
}