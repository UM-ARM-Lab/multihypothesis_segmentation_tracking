//
// Created by kunhuang on 6/16/20.
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
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/util/package_paths.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/SegmentationTreeSampler.h"
#include "mps_voxels/ValueTree_impl.hpp"
#include "mps_voxels/relabel_tree_image.hpp"
#include "mps_voxels/Ultrametric.h"
#include "mps_voxels/image_output.h"
#include "mps_voxels/voxel_recombination.h"

using namespace mps;
using VoxelColormap = std::map<mps::VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>;

const std::string testDirName = "package://mps_test_data/";
const std::string expDirName = "2020-06-31/";
const std::string logDir = parsePackageURL(testDirName);
const int numParticles = 5;
const int generation = 4;
const ExperimentDir checkpointDir;
const double table_height = 0.03; //0.03 for simulation

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

		/////////////////////////////////////////////
		//// Load initial particles
		/////////////////////////////////////////////
		for (int i=0; i<particleFilter->numParticles; ++i)
		{
			Particle p;
			p.state = std::make_shared<Particle::ParticleData>(particleFilter->voxelRegion);
			DataLog loader(logDir + expDirName + checkpointDir.checkpoints[3] + "/particle_" + std::to_string(generation) + "_" + std::to_string(i) + ".bag",
			               {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("particle");
			*p.state = loader.load<OccupancyData>("particle");
			p.particle.id = i;
			particleFilter->particles.push_back(p);
			particleFilter->particles[i].state->uniqueObjectLabels = getUniqueObjectLabels(particleFilter->particles[i].state->vertexState);
			std::cerr << "Successfully loaded." << std::endl;
		}
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_refine");
	ros::NodeHandle nh, pnh("~");

	ParticleFilterTestFixture fixture;
	fixture.SetUp();

	ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
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

/*
	for (int i=0; i<fixture.particleFilter->numParticles; ++i)
	{
		Particle p;
		p.state = std::make_shared<Particle::ParticleData>(fixture.particleFilter->voxelRegion);
		DataLog loader(logDir + expDirName + "particle_" + std::to_string(0) + "_" + std::to_string(i) + ".bag",
		               {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("particle");
		*p.state = loader.load<OccupancyData>("particle");
		p.particle.id = i;
		std::cerr << "Successfully loaded." << std::endl;

		std_msgs::Header header;
		header.frame_id = scenario->mapServer->getWorldFrame();
		header.stamp = ros::Time::now();
		auto pfnewmarker = mps::visualize(*p.state, header, rng);
		visualPub.publish(pfnewmarker);
		std::cerr << "Initial state particle " << i << " shown!" << std::endl;
		usleep(500000);
	}
*/


/*
	//// Compute new scene
	ros::Time finalTime = fixture.motionData.rgb.rbegin()->first;
	std::shared_ptr<Scene> newScene = computeSceneFromSensorHistorian(scenario, fixture.motionData, finalTime, scenario->worldFrame);

	/////////////////////////////////////////////
	//// Compute new sceneOctree
	/////////////////////////////////////////////
	//// Look up transformation
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


	/////////////////////////////////////////////
	//// Compute weights
	/////////////////////////////////////////////
	fixture.particleFilter->applyMeasurementModel(newScene);
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		std::cerr << "Particle " << i << " weight: " << fixture.particleFilter->particles[i].weight << std::endl;
	}


	/////////////////////////////////////////////
	//// Free space refinement
	/////////////////////////////////////////////
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		refineParticleFreeSpace(fixture.particleFilter->particles[i], sceneOctree, table_height);
		std_msgs::Header header;
		header.frame_id = scenario->mapServer->getWorldFrame();
		header.stamp = ros::Time::now();
		auto pfnewmarker = mps::visualize(*fixture.particleFilter->particles[i].state, header, rng);
		visualPub.publish(pfnewmarker);
		std::cerr << "Free-space refined predicted state particle " << i << " shown!" << std::endl;
		usleep(500000);
	}

	/////////////////////////////////////////////
	//// Compute weights for old particles X_t' after refinement
	/////////////////////////////////////////////
	fixture.particleFilter->applyMeasurementModel(newScene);
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		std::cerr << "Particle " << i << " after refinement weight: " << fixture.particleFilter->particles[i].weight << std::endl;
	}
*/

	/// Visualize old particles X_t'
/*
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		std_msgs::Header header; header.frame_id = scenario->mapServer->getWorldFrame(); header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*fixture.particleFilter->particles[i].state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "old particle " << i << " shown!" << std::endl;
		usleep(500000);

		const VoxelRegion& region = *fixture.particleFilter->particles[i].state->voxelRegion;
		assert(fixture.particleFilter->particles[i].state->vertexState.size() == region.num_vertices());
		VoxelRegion::VertexLabels outputState(region.num_vertices(), VoxelRegion::FREE_SPACE);

//#pragma omp parallel for
		for (size_t j = 0; j < fixture.particleFilter->particles[i].state->vertexState.size(); ++j)
		{
			if (fixture.particleFilter->particles[i].state->vertexState[j] != VoxelRegion::FREE_SPACE)
			{
//					const auto& T = action.at(ObjectIndex(inputParticle.state->vertexState[i])).tf;
				VoxelRegion::vertex_descriptor vd = region.vertex_at(j);
				Eigen::Vector3d originalCoord = vertexDescpToCoord(region.resolution, region.regionMin, vd);
				Eigen::Vector3d newCoord = originalCoord;
//				newCoord.x() = originalCoord.x() + 3 * region.resolution;
//				newCoord.z() = originalCoord.z() + 3 * region.resolution;


				if (!region.isInRegion(newCoord)) continue;

				VoxelRegion::vertex_descriptor newVD = coordToVertexDesc(region.resolution, region.regionMin, newCoord);
				auto index = region.index_of(newVD);
				outputState[index] = fixture.particleFilter->particles[i].state->vertexState[j];
				outputState[j] = fixture.particleFilter->particles[i].state->vertexState[j];
			}
		}

		Particle outputParticle;
		outputParticle.state = std::make_shared<OccupancyData>(fixture.particleFilter->particles[i].state->voxelRegion, outputState);
		outputParticle.particle = fixture.particleFilter->particles[i].particle;


		pfMarkers = mps::visualize(*outputParticle.state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "old particle moved " << i << " shown!" << std::endl;
		usleep(500000);

		fixture.particleFilter->particles[i] = outputParticle;
	}
*/

	/////////////////////////////////////////////
	//// Free space refinement
	/////////////////////////////////////////////
	//// Look up transformation
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

/*
	{
		visualization_msgs::MarkerArray ma = visualizeOctree(sceneOctree, scenario->worldFrame);
		for (visualization_msgs::Marker& m : ma.markers)
		{
			m.ns = "map";
		}
		visualPub.publish(ma);
	}
*/

	std::vector<double> historyParticleQuality;
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		VoxelColormap cmap;
		auto colors = dispersedColormap(fixture.particleFilter->particles[i].state->uniqueObjectLabels.size());
		int colorID = 0;
		for (const auto& label : fixture.particleFilter->particles[i].state->uniqueObjectLabels)
		{
			cmap.emplace(label.id, colors[colorID]);
			++colorID;
		}

/*
		Particle p;
		p.state = std::make_shared<Particle::ParticleData>(fixture.particleFilter->voxelRegion);
		DataLog loader(logDir + expDirName + "particle_" + std::to_string(0) + "_" + std::to_string(i) + ".bag",
		               {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("particle");
		*p.state = loader.load<OccupancyData>("particle");
		p.particle.id = i;
		std::cerr << "Successfully loaded." << std::endl;
*/

/*
		{
			std_msgs::Header header;
			header.frame_id = scenario->mapServer->getWorldFrame();
			header.stamp = ros::Time::now();
			auto pfnewmarker = mps::visualize(*p.state, header, cmap);
			visualPub.publish(pfnewmarker);
			std::cerr << "Initial state particle " << i << " shown!" << std::endl;
			usleep(500000);
		}
*/

		std_msgs::Header header; header.frame_id = scenario->mapServer->getWorldFrame(); header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*fixture.particleFilter->particles[i].state, header, cmap);
		visualPub.publish(pfMarkers);
		std::cerr << "old particle " << i << " shown!" << std::endl;
//		usleep(500000);

		std::pair<int, int> refineResults = refineParticleFreeSpace(fixture.particleFilter->particles[i], sceneOctree, table_height);
		double quality = static_cast<double>(refineResults.first) / static_cast<double>(refineResults.second);
		historyParticleQuality.push_back(quality);
		std::cerr << "Particle quality " << 1 - quality << std::endl;

		header.frame_id = scenario->mapServer->getWorldFrame();
		header.stamp = ros::Time::now();
		auto pfnewmarker = mps::visualize(*fixture.particleFilter->particles[i].state, header, cmap);
		visualPub.publish(pfnewmarker);
		std::cerr << "Free-space refined predicted state particle " << i << " shown!" << std::endl;
//		usleep(500000);
		{
			DataLog logger(logDir + expDirName + checkpointDir.checkpoints[4] + "/particle_" + std::to_string(generation) + "_" + std::to_string(i) + ".bag");
			logger.activeChannels.insert("particle");
			logger.log<OccupancyData>("particle", *fixture.particleFilter->particles[i].state);
			std_msgs::Float64 weightMsg; weightMsg.data = 1 - quality;
			logger.activeChannels.insert("weight");
			logger.log<std_msgs::Float64>("weight", weightMsg);
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
		DataLog loader(logDir + expDirName + checkpointDir.checkpoints[0] + "/particle_" +
		               std::to_string(generation)+ "_" + std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("particle");
		*p.state = loader.load<OccupancyData>("particle");
		p.particle.id = i;
		newSamples.push_back(p);
		newSamples[i].state->uniqueObjectLabels = getUniqueObjectLabels(newSamples[i].state->vertexState);
		std::cerr << "Successfully loaded measurement particle_" << generation << "_" << i <<" with quality " << p.weight << std::endl;
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
//		usleep(500000);
	}

	for (int newParticleID = 0; newParticleID<static_cast<int>(newSamples.size()); ++newParticleID)
	{
		for (int oldParticleID = 0; oldParticleID<fixture.particleFilter->numParticles; ++oldParticleID)
		{
			std::vector<const VoxelRegion::VertexLabels*> toCombine;
			toCombine.emplace_back(&newSamples[newParticleID].state->vertexState);
//			toCombine.emplace_back(&newSamples[newParticleID+1].state->vertexState);
			toCombine.emplace_back(&fixture.particleFilter->particles[oldParticleID].state->vertexState);

			VoxelConflictResolver resolver(fixture.particleFilter->voxelRegion, toCombine);
			std::cerr << toCombine[0]->size() << " " << toCombine[1]->size() << std::endl;

			std::cerr << "Mixing new particle " << newParticleID << " and old particle " << oldParticleID << std::endl;
			std::vector<Particle> mixParticles;
			for (int nsample = 0; nsample < 5; ++nsample)
			{
				auto structure = resolver.sampleStructure(rng);
				auto V = resolver.sampleGeometry(toCombine, structure, rng);

				Particle inputParticle;
				inputParticle.state = std::make_shared<OccupancyData>(newSamples[newParticleID].state->voxelRegion, V);
				inputParticle.particle = newSamples[newParticleID].particle;
				assert(!inputParticle.state->uniqueObjectLabels.empty());

				VoxelColormap cmap;
				auto colors = dispersedColormap(inputParticle.state->uniqueObjectLabels.size());
				int colorID = 0;
				for (const auto& label : inputParticle.state->uniqueObjectLabels)
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

				while(!isconverge && numFilter < 100)
				{
					inputParticle = filteringParticle(inputParticle, isconverge);
					auto pfMarkers = mps::visualize(*inputParticle.state, header, cmap);
					visualPub.publish(pfMarkers);
					std::cerr << "Mixed particle filtered " << nsample << " shown!" << std::endl;
//					usleep(50000);
					numFilter++;
				}
				mixParticles.push_back(inputParticle);
				{
					DataLog logger(logDir + expDirName + checkpointDir.checkpoints[1] + "/particle_" + std::to_string(generation) + "_" +
					               std::to_string(newParticleID) + "_" + std::to_string(oldParticleID) + "_" + std::to_string(nsample) + ".bag");
					logger.activeChannels.insert("particle");
					logger.log<OccupancyData>("particle", *inputParticle.state);
					ROS_INFO_STREAM("Logged mixed particle " << newParticleID << " " << oldParticleID);
				}

//				usleep(2000000);
			}

			//// Remixing sampled merging result
/*
			std::vector<const VoxelRegion::VertexLabels*> toCombineAgain;
			for (auto& p : mixParticles)
			{
				toCombineAgain.emplace_back(&p.state->vertexState);
			}
			VoxelConflictResolver resolver2(fixture.particleFilter->voxelRegion, toCombineAgain);
			for (int iter = 0; iter < 5; ++iter)
			{
				auto structure = resolver2.sampleStructure(scenario->rng());
				auto V = resolver2.sampleGeometry(toCombineAgain, structure, scenario->rng());

				Particle inputParticle;
				inputParticle.state = std::make_shared<OccupancyData>(newSamples[newParticleID].state->voxelRegion, V);
				inputParticle.particle = newSamples[newParticleID].particle;
				assert(!inputParticle.state->uniqueObjectLabels.empty());

				std_msgs::Header header;
				header.frame_id = scenario->mapServer->getWorldFrame();
				header.stamp = ros::Time::now();
				auto pfnewmarker = mps::visualize(*fixture.particleFilter->voxelRegion, V, header, rng);
				visualPub.publish(pfnewmarker);
				std::cerr << "Mixed particle " << iter << " shown!" << std::endl;
				sleep(1);
			}
*/
		}
	}

	return 0;
}