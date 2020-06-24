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

#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_sensor_history.h"
#include "mps_voxels/logging/log_scene.h"
#include "mps_voxels/logging/log_segmentation_info.h"

#include "mps_voxels/visualization/visualize_occupancy.h"
#include "mps_voxels/visualization/visualize_voxel_region.h"

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

const std::string testDirName = "package://mps_test_data/";
const std::string expDirName = "2020-06-07T08:55:02.095309/";
const std::string logDir = parsePackageURL(testDirName);


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
		                                                  scenario->maxExtent.head<3>(), 2);

		/////////////////////////////////////////////
		//// Load initial scene data
		/////////////////////////////////////////////
		{
			DataLog loader(logDir + expDirName + "buffer_" + std::to_string(0) + ".bag", {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("buffer");
			motionData = loader.load<SensorHistoryBuffer>("buffer");
			std::cerr << "Successfully loaded buffer." << std::endl;
		}
		std::cerr << "number of frames: " << motionData.rgb.size() << std::endl;

		/////////////////////////////////////////////
		//// Load initial particles
		/////////////////////////////////////////////
		int generation = 1;
		for (int i=0; i<particleFilter->numParticles; ++i)
		{
			Particle p;
			p.state = std::make_shared<Particle::ParticleData>(particleFilter->voxelRegion);
			DataLog loader(logDir + expDirName + "particle_" + std::to_string(generation) + "_" + std::to_string(i) + ".bag",
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
	ros::init(argc, argv, "test_pf");
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

/*
	/////////////////////////////////////////////
	//// Compute fundamental nodes
	/////////////////////////////////////////////
	SegmentationTreeSampler treeSampler(newScene->segInfo);
	std::vector<std::pair<double, SegmentationCut>> segmentationSamples = treeSampler.sample(scenario->rng(), 10, true);
	std::vector<mps::tree::TreeCut> samples;

	for (size_t i = 0; i<segmentationSamples.size(); i++)
	{
		auto& ss = segmentationSamples[i];
		samples.push_back(ss.second.cut);
	}
	mps::tree::TreeCut fundamentalNodes = getFundamentalNodes(treeSampler.vt, samples);
	if (!isCutComplete(treeSampler.vt, fundamentalNodes)) std::cerr << "Wrong fundamentalNodes!" << std::endl;

	SegmentationCut fundamentalSeg = treeSampler.treeCut2segCut(fundamentalNodes);
	cv::imwrite(logDir + expDirName + "fundamentalNodes.png", colorByLabel(fundamentalSeg.segmentation.objectness_segmentation->image));

	Particle fundamentalParticle;

	fundamentalParticle.state = std::make_shared<OccupancyData>(fixture.particleFilter->voxelRegion);

	const auto segInfo = fundamentalSeg.segmentation;

	bool execSegmentation = SceneProcessor::performSegmentation(*newScene, segInfo.objectness_segmentation->image,
	                                                            *fundamentalParticle.state);
	if (!execSegmentation)
	{
		std::cerr << "New Optimal Particle failed to segment." << std::endl;
	}

	//// without shape completion
	bool getSurface = SceneProcessor::buildObjSurface(*newScene, *fundamentalParticle.state);
	if (!getSurface)
	{
		std::cerr << "fundamentalParticle failed to build surface." << std::endl;
	}

	fundamentalParticle.state->vertexState =
		fixture.particleFilter->voxelRegion->objectsToSubRegionVoxelLabel(fundamentalParticle.state->objects,
		                                                                  newScene->minExtent.head<3>());
	fundamentalParticle.state->uniqueObjectLabels =
		getUniqueObjectLabels(fundamentalParticle.state->vertexState);

	for (int i = 0; i<10; i++)
	{
		std_msgs::Header header;
		header.frame_id = scenario->mapServer->getWorldFrame();
		header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*fundamentalParticle.state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "fundamentalParticle shown!" << std::endl;
		sleep(2);
	}
*/

	/// Visualize old particles X_t'
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		std_msgs::Header header; header.frame_id = scenario->mapServer->getWorldFrame(); header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*fixture.particleFilter->particles[i].state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "State particle " << i << " shown!" << std::endl;
		usleep(500000);
	}

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
		refineParticleFreeSpace(fixture.particleFilter->particles[i], sceneOctree);
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

/*
	/////////////////////////////////////////////
	//// Generate optimal particle for newScene
	/////////////////////////////////////////////
	fixture.particleFilter->newSceneParticle.state = std::make_shared<OccupancyData>(fixture.particleFilter->voxelRegion);

	const auto segInfo = newScene->segInfo;

	bool execSegmentation = SceneProcessor::performSegmentation(*newScene, segInfo->objectness_segmentation->image,
	                                                            *fixture.particleFilter->newSceneParticle.state);
	if (!execSegmentation)
	{
		std::cerr << "New Optimal Particle failed to segment." << std::endl;
	}

	bool getCompletion = SceneProcessor::buildObjects(*newScene, *fixture.particleFilter->newSceneParticle.state);
	if (!getCompletion)
	{
		std::cerr << "New Optimal Particle failed to shape complete." << std::endl;
	}

	fixture.particleFilter->newSceneParticle.state->vertexState =
		fixture.particleFilter->voxelRegion->objectsToSubRegionVoxelLabel(fixture.particleFilter->newSceneParticle.state->objects,
		                                                                  newScene->minExtent.head<3>());
	fixture.particleFilter->newSceneParticle.state->uniqueObjectLabels =
		getUniqueObjectLabels(fixture.particleFilter->newSceneParticle.state->vertexState);

	{
		std_msgs::Header header;
		header.frame_id = scenario->mapServer->getWorldFrame();
		header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*fixture.particleFilter->newSceneParticle.state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "New optimal state particle shown!" << std::endl;
		sleep(5);
	}
*/

	/////////////////////////////////////////////
	//// Mixturing new and old particles
	/////////////////////////////////////////////
	auto newSamples = fixture.particleFilter->createParticlesFromSegmentation(newScene, fixture.particleFilter->numParticles);
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
			std::vector<const VoxelRegion::VertexLabels*> toCombine;
			toCombine.emplace_back(&newSamples[newParticleID].state->vertexState);
			toCombine.emplace_back(&fixture.particleFilter->particles[oldParticleID].state->vertexState);

			VoxelConflictResolver resolver(fixture.particleFilter->voxelRegion, toCombine);

			std::cerr << "Mixing new particle " << newParticleID << " and old particle " << oldParticleID << std::endl;
			for (int iter = 0; iter < 5; ++iter)
			{
				auto structure = resolver.sampleStructure(scenario->rng());
				auto V = resolver.sampleGeometry(toCombine, structure, scenario->rng());

				std_msgs::Header header;
				header.frame_id = scenario->mapServer->getWorldFrame();
				header.stamp = ros::Time::now();
				auto pfnewmarker = mps::visualize(*fixture.particleFilter->voxelRegion, V, header, scenario->rng());
				visualPub.publish(pfnewmarker);
				std::cerr << "Mixed particle " << iter << " shown!" << std::endl;
				sleep(3);
			}
		}
	}

	return 0;
}