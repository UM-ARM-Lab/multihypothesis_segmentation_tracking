#include <iostream>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

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
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/util/package_paths.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/SegmentationTreeSampler.h"
#include "mps_voxels/relabel_tree_image.hpp"
#include "mps_voxels/image_output.h"
#include "mps_voxels/voxel_recombination.h"

#if HAS_CUDA_SIFT
#include "mps_voxels/CudaTracker.h"
#endif

#define USE_CPU_SIFT true
#define USE_HISTORYTRACKER false


using namespace mps;
using VoxelColormap = std::map<mps::VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>;

const int numIter = 4; /// # buffer bags
const int numParticles = 5;
const std::vector<int> particleIDs = {1,6,7,8,9};
const int generation = 3;
const std::string testDirName = "package://mps_test_data/";
const std::string expDirName = "2020-06-30/";
const std::string logDir = parsePackageURL(testDirName);
const std::string trackingFilename = logDir + expDirName + "dense_track_" + std::to_string(generation) + "_" + std::to_string(0) + ".bag";
const ExperimentDir checkpointDir;

class ParticleFilterTestFixture
{
public:
	std::unique_ptr<ParticleFilter> particleFilter;
	std::unique_ptr<robot_model_loader::RobotModelLoader> mpLoader;
	robot_model::RobotModelPtr pModel;
	std::shared_ptr<Scenario> scenario;
	std::shared_ptr<Scene> initialScene;
	SensorHistoryBuffer motionData;
	std::unique_ptr<Tracker> sparseTracker;
	std::unique_ptr<DenseTracker> denseTracker;

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
#if HAS_CUDA_SIFT
		sparseTracker = std::make_unique<CudaTracker>(opts);
#else
		sparseTracker = std::make_unique<Tracker>(opts);
		sparseTracker->track_options.featureRadius = 200.0f;
		sparseTracker->track_options.pixelRadius = 1000.0f;
		sparseTracker->track_options.meterRadius = 1.0f;
#endif

#if USE_HISTORYTRACKER
		denseTracker = std::make_unique<HistoryTracker>(trackingFilename);
#else
		denseTracker = std::make_unique<SiamTracker>(logDir + expDirName);
#endif

		double resolution = 0.010;
		pnh.getParam("resolution", resolution);
		particleFilter = std::make_unique<ParticleFilter>(scenario, resolution,
		                                                  scenario->minExtent.head<3>(),
		                                                  scenario->maxExtent.head<3>(), numParticles);


		/////////////////////////////////////////////
		//// Load initial particles
		/////////////////////////////////////////////
		//// Caution: logged particles are sampled from new scene
		for (int i=0; i<particleFilter->numParticles; ++i)
		{
			Particle p;
			p.state = std::make_shared<Particle::ParticleData>(particleFilter->voxelRegion);
			if (generation == 0)
			{
				DataLog loader(logDir + expDirName + checkpointDir.checkpoints[0] + "/particle_" + std::to_string(generation) + "_" +
				               std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
				loader.activeChannels.insert("particle");
				*p.state = loader.load<OccupancyData>("particle");
				p.particle.id = i;
			}
			else
			{
				DataLog loader(logDir + expDirName + checkpointDir.checkpoints[2] + "/particle_" + std::to_string(generation) + "_" +
				               std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
				loader.activeChannels.insert("particle");
				*p.state = loader.load<OccupancyData>("particle");
				p.particle.id = i;
			}
			particleFilter->particles.push_back(p);
			particleFilter->particles[i].state->uniqueObjectLabels = getUniqueObjectLabels(particleFilter->particles[i].state->vertexState);
			std::cerr << "Successfully loaded." << std::endl;
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pfPipeline");
	ros::NodeHandle nh, pnh("~");
//	if (!nh.hasParam("/use_sim_time"))
//	{
//		ROS_INFO("No param named '/use_sim_time'");
//	}
//	nh.setParam("/use_sim_time", false);

	ParticleFilterTestFixture fixture;
	fixture.SetUp();

	ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("pfPipeline", 1, true);
	std::mt19937 rng;

	/////////////////////////////////////////////
	//// Initialize mapServer
	/////////////////////////////////////////////
	std::shared_ptr<Scenario> scenario = fixture.scenario;
	scenario->mapServer = std::make_shared<LocalOctreeServer>(0.01, "table_surface");
	octomap::point3d tMin(scenario->minExtent.x(), scenario->minExtent.y(), scenario->minExtent.z());
	octomap::point3d tMax(scenario->maxExtent.x(), scenario->maxExtent.y(), scenario->maxExtent.z());
	scenario->mapServer->m_octree->setBBXMin(tMin);
	scenario->mapServer->m_octree->setBBXMax(tMax);


	for (int generationID = generation; generationID < generation+1; ++generationID)
	{
		/// Visualize old particles X_t
		for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
		{
			std_msgs::Header header; header.frame_id = scenario->mapServer->getWorldFrame(); header.stamp = ros::Time::now();
			auto pfMarkers = mps::visualize(*fixture.particleFilter->particles[i].state, header, rng);
			visualPub.publish(pfMarkers);
			std::cerr << "Old particle " << i << " shown!" << std::endl;
//			usleep(500000);
		}

		std::cout << "--------------------------------------------------------- Iteration " << generationID <<
		          " ---------------------------------------------------------" << std::endl;
		/////////////////////////////////////////////
		//// Load motion data
		/////////////////////////////////////////////
		{
			DataLog loader(logDir + expDirName + "buffer_" + std::to_string(generationID) + ".bag", {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("buffer");
			fixture.motionData = loader.load<SensorHistoryBuffer>("buffer");
			std::cerr << "Successfully loaded buffer." << std::endl;
		}
		std::cerr << "Number of frames loaded: " << fixture.motionData.rgb.size() << std::endl;

		/////////////////////////////////////////////
		//// Compute And Apply ActionModel
		/////////////////////////////////////////////
		fixture.particleFilter->computeAndApplyActionModel(fixture.motionData, fixture.sparseTracker, fixture.denseTracker);
		for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
		{
			std_msgs::Header header; header.frame_id = scenario->mapServer->getWorldFrame(); header.stamp = ros::Time::now();
			auto pfMarkers = mps::visualize(*fixture.particleFilter->particles[i].state, header, rng);
			visualPub.publish(pfMarkers);
			std::cerr << "X_t' " << i << " shown!" << std::endl;
//			usleep(500000);
			{
				DataLog logger(logDir + expDirName + checkpointDir.checkpoints[3] + "/particle_" + std::to_string(generationID+1) + "_" + std::to_string(i) + ".bag");
				logger.activeChannels.insert("particle");
				logger.log<OccupancyData>("particle", *fixture.particleFilter->particles[i].state);
				ROS_INFO_STREAM("Logged X_t' " << i);
			}
		}


		/////////////////////////////////////////////
		//// Free space refinement
		/////////////////////////////////////////////
		//// Look up transformation
/*
		moveit::Pose worldTcamera;
		const auto& cameraModel = fixture.motionData.cameraModel;
		const ros::Time queryTime = ros::Time(0); // buffer.rgb.begin()->first;
		const ros::Duration timeout = ros::Duration(5.0);
		std::string tfError;
		bool canTransform = fixture.motionData.tfs->canTransform(scenario->worldFrame, cameraModel.tfFrame(), queryTime, &tfError);
		if (!canTransform) // ros::Duration(5.0)
		{
			ROS_ERROR_STREAM("Failed to look up transform between '" << scenario->worldFrame << "' and '"
			                                                         << cameraModel.tfFrame() << "' with error '"
			                                                         << tfError << "'.");
			throw std::runtime_error("Sadness.");
		}
		tf::StampedTransform cameraFrameInTableCoordinates;
		const auto temp = fixture.motionData.tfs->lookupTransform(cameraModel.tfFrame(), scenario->worldFrame, queryTime);
		tf::transformStampedMsgToTF(temp, cameraFrameInTableCoordinates);
		tf::transformTFToEigen(cameraFrameInTableCoordinates.inverse(), worldTcamera);

		//// get new sceneOctree
		pcl::PointCloud<PointT>::Ptr finalPC = imagesToCloud(fixture.motionData.rgb.rbegin()->second->image,
		                                                     fixture.motionData.depth.rbegin()->second->image, fixture.motionData.cameraModel);
		scenario->mapServer->insertCloud(finalPC, worldTcamera);
		octomap::OcTree* sceneOctree = scenario->mapServer->getOctree();

		for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
		{
			refineParticleFreeSpace(fixture.particleFilter->particles[i], sceneOctree, 0.03);
			std_msgs::Header header;
			header.frame_id = scenario->mapServer->getWorldFrame();
			header.stamp = ros::Time::now();
			auto pfnewmarker = mps::visualize(*fixture.particleFilter->particles[i].state, header, rng);
			visualPub.publish(pfnewmarker);
			std::cerr << "Free-space refined predicted state particle " << i << " shown!" << std::endl;
			usleep(500000);
			{
				DataLog logger(logDir + expDirName + resultDir + "particlePrimeRefined_" + std::to_string(generationID+1) + "_" + std::to_string(i) + ".bag");
				logger.activeChannels.insert("particle");
				logger.log<OccupancyData>("particle", *fixture.particleFilter->particles[i].state);
				ROS_INFO_STREAM("Logged X_t' refined " << i);
			}
		}

		/////////////////////////////////////////////
		//// Mixturing new and old particles
		/////////////////////////////////////////////
		//// Sample from the new scene by loading particles
		std::vector<Particle> newSamples;
		for (int i=0; i<fixture.particleFilter->numParticles; ++i)
		{
			Particle p;
			p.state = std::make_shared<Particle::ParticleData>(fixture.particleFilter->voxelRegion);
			DataLog loader(logDir + expDirName + "particle_" + std::to_string(generationID+1)+ "_" + std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("particle");
			*p.state = loader.load<OccupancyData>("particle");
			p.particle.id = i;
			newSamples.push_back(p);
			newSamples[i].state->uniqueObjectLabels = getUniqueObjectLabels(newSamples[i].state->vertexState);
			std::cerr << "Successfully loaded measurement particle " << generationID+1 << " " << i << std::endl;
		}
		//// TODO: if more particles, generate scene
//		auto newSamples = fixture.particleFilter->createParticlesFromSegmentation(newScene, fixture.particleFilter->numParticles);


		/// Visualize new particles X_t''
		for (int i = 0; i<static_cast<int>(newSamples.size()); ++i)
		{
			std_msgs::Header header; header.frame_id = scenario->mapServer->getWorldFrame(); header.stamp = ros::Time::now();
			auto pfMarkers = mps::visualize(*newSamples[i].state, header, rng);
			visualPub.publish(pfMarkers);
			std::cerr << "New particle " << i << " shown!" << std::endl;
			usleep(500000);
		}

		for (int newParticleID = 0; newParticleID<static_cast<int>(newSamples.size()); ++newParticleID)
		{
			for (int oldParticleID = 0; oldParticleID<fixture.particleFilter->numParticles; ++oldParticleID)
			{
				std::vector<const VoxelRegion::VertexLabels *> toCombine;
				toCombine.emplace_back(&newSamples[newParticleID].state->vertexState);
				toCombine.emplace_back(&fixture.particleFilter->particles[oldParticleID].state->vertexState);

				VoxelConflictResolver resolver(fixture.particleFilter->voxelRegion, toCombine);

				std::cerr << "Mixing new particle " << newParticleID << " and old particle " << oldParticleID
				          << std::endl;
				std::vector<Particle> mixParticles;
				for (int nsample = 0; nsample < 5; ++nsample)
				{
					auto structure = resolver.sampleStructure(scenario->rng());
					auto V = resolver.sampleGeometry(toCombine, structure, scenario->rng());

					Particle inputParticle;
					inputParticle.state = std::make_shared<OccupancyData>(newSamples[newParticleID].state->voxelRegion,
					                                                      V);
					inputParticle.particle = newSamples[newParticleID].particle;
					assert(!inputParticle.state->uniqueObjectLabels.empty());

					VoxelColormap cmap;
					auto colors = dispersedColormap(inputParticle.state->uniqueObjectLabels.size());
					int colorID = 0;
					for (const auto &label : inputParticle.state->uniqueObjectLabels)
					{
						cmap.emplace(label.id, colors[colorID]);
						++colorID;
					}

					std_msgs::Header header;
					header.frame_id = scenario->mapServer->getWorldFrame();
					header.stamp = ros::Time::now();
					auto pfnewmarker = mps::visualize(*fixture.particleFilter->voxelRegion, V, header, cmap);
					visualPub.publish(pfnewmarker);
					std::cerr << "Mixed particle " << nsample << " shown!" << std::endl;
//				sleep(1);

					bool isconverge = false;
					int numFilter = 0;

					while (!isconverge && numFilter < 100)
					{
						inputParticle = filteringParticle(inputParticle, isconverge);
*/
/*
						auto pfMarkers = mps::visualize(*inputParticle.state, header, cmap);
						visualPub.publish(pfMarkers);
						std::cerr << "Mixed particle filtered " << nsample << " shown!" << std::endl;
						usleep(100000);
*//*

						numFilter++;
					}
					mixParticles.push_back(inputParticle);
					{
						DataLog logger(logDir + expDirName + resultDir + "mixture/" + "particleMixed_" + std::to_string(generationID+1) + "_" +
						               std::to_string(newParticleID) + "_" + std::to_string(oldParticleID) + "_" + std::to_string(nsample) + ".bag");
						logger.activeChannels.insert("particle");
						logger.log<OccupancyData>("particle", *inputParticle.state);
						ROS_INFO_STREAM("Logged mixed particle " << newParticleID << " " << oldParticleID);
					}

//					usleep(2000000);
				}
			}
		}
*/

/*
		/// TODO: overwrite particleFilter.particles with mixture result
		fixture.particleFilter->particles = newSamples;
		for (int id = 0; id < fixture.particleFilter->numParticles; ++id)
		{
			DataLog logger(logDir + expDirName + resultDir + "particleMixed_" + std::to_string(generationID+1) + "_" + std::to_string(id) + ".bag");
			logger.activeChannels.insert("particle");
			logger.log<OccupancyData>("particle", *fixture.particleFilter->particles[id].state);
			ROS_INFO_STREAM("Logged X_t mixed " << id);
		}
*/
	}

	return 0;
}