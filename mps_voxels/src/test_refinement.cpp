//
// Created by kunhuang on 6/16/20.
//

#include <iostream>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>

#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_sensor_history.h"
#include "mps_voxels/logging/log_scene.h"
#include "mps_voxels/logging/log_segmentation_info.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/ParticleFilter.h"
#include "mps_voxels/visualization/visualize_occupancy.h"
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/util/package_paths.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/octree_utils.h"

#include <moveit/robot_model_loader/robot_model_loader.h>

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
	std::default_random_engine rng;

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
	/// Visualize particles
	/////////////////////////////////////////////
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		std_msgs::Header header; header.frame_id = scenario->mapServer->getWorldFrame(); header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*fixture.particleFilter->particles[i].state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "State particle " << i << " shown!" << std::endl;
		sleep(2);
	}

	/////////////////////////////////////////////
	//// Compute weights
	/////////////////////////////////////////////
	fixture.particleFilter->applyMeasurementModel(newScene);
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		std::cerr << "Particle " << i << " weight: " << fixture.particleFilter->particles[i].weight << std::endl;
	}

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


	/////////////////////////////////////////////
	//// Compute new sceneOctree
	/////////////////////////////////////////////
	pcl::PointCloud<PointT>::Ptr finalPC = imagesToCloud(fixture.motionData.rgb.rbegin()->second->image,
	                                                     fixture.motionData.depth.rbegin()->second->image, fixture.motionData.cameraModel);
	scenario->mapServer->insertCloud(finalPC, worldTcamera);
	octomap::OcTree* sceneOctree = scenario->mapServer->getOctree();

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
		sleep(2);
	}

	/////////////////////////////////////////////
	//// Compute weights
	/////////////////////////////////////////////
	fixture.particleFilter->applyMeasurementModel(newScene);
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		std::cerr << "Particle " << i << " after refinement weight: " << fixture.particleFilter->particles[i].weight << std::endl;
	}

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

	/////////////////////////////////////////////
	//// Refinement based on new optimal particle
	/////////////////////////////////////////////
//	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
//	{
//		refineParticleFreeSpace(fixture.particleFilter->particles[i], sceneOctree);
//		std_msgs::Header header;
//		header.frame_id = scenario->mapServer->getWorldFrame();
//		header.stamp = ros::Time::now();
//		auto pfnewmarker = mps::visualize(*fixture.particleFilter->particles[i].state, header, rng);
//		visualPub.publish(pfnewmarker);
//		std::cerr << "Refinement based on new optimal particle " << i << " shown!" << std::endl;
//		sleep(2);
//	}

	//TODO: 	JaccardMatch3D(const OccupancyData& labels1, const OccupancyData& labels2);


	return 0;
}