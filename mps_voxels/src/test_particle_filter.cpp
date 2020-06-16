//
// Created by kunhuang on 2/12/20.
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

#if HAS_CUDA_SIFT
#include "mps_voxels/CudaTracker.h"
#endif

#define USE_CPU_SIFT true

#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace mps;

const std::string testDirName = "package://mps_test_data/";
const std::string expDirName = "2020-06-07T08:55:02.095309/";
const std::string logDir = parsePackageURL(testDirName);
const std::string trackingFilename = logDir + expDirName + "dense_track_" + std::to_string(0) + "_" + std::to_string(0) + ".bag";

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
//		denseTracker = std::make_unique<SiamTracker>();
		denseTracker = std::make_unique<HistoryTracker>(trackingFilename);

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
//		SegmentationInfo seg_out;
//		{
//			DataLog loader(logDir + expDirName + "segInfo_" + std::to_string(0) + ".bag", {}, rosbag::bagmode::Read);
//			loader.activeChannels.insert("segInfo");
//			seg_out = loader.load<SegmentationInfo>("segInfo");
//			std::cerr << "Successfully loaded segInfo." << std::endl;
//		}

		ros::Time initialTime = motionData.rgb.begin()->first;
		initialScene = computeSceneFromSensorHistorian(scenario, motionData, initialTime, scenario->worldFrame);
//		initialScene->roi = initialScene->segInfo->roi;

		std::cerr << "initialScene->segInfo->roi: " << initialScene->segInfo->roi.x << " " << initialScene->segInfo->roi.y << " " << initialScene->segInfo->roi.height << " " << initialScene->segInfo->roi.width << std::endl;
		std::cerr << "initialScene->roi: " << initialScene->roi.x << " " << initialScene->roi.y << " " << initialScene->roi.height << " " << initialScene->roi.width << std::endl;

		/////////////////////////////////////////////
		//// Load initial particles
		/////////////////////////////////////////////
//		particleFilter->initializeParticles(initialScene);

		for (int i=0; i<particleFilter->numParticles; ++i)
		{
			Particle p;
			p.state = std::make_shared<Particle::ParticleData>(particleFilter->voxelRegion);
			DataLog loader(logDir + expDirName + "particle_0_" + std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("particle");
			*p.state = loader.load<OccupancyData>("particle");
			p.particle.id = i;
			particleFilter->particles.push_back(p);
			particleFilter->particles[i].state->uniqueObjectLabels = getUniqueObjectLabels(particleFilter->particles[i].state->vertexState);
			std::cerr << "Successfully loaded." << std::endl;
		}
		// Load motion data
	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_pf");
	ros::NodeHandle nh, pnh("~");
//	if (!nh.hasParam("/use_sim_time"))
//	{
//		ROS_INFO("No param named '/use_sim_time'");
//	}
//	nh.setParam("/use_sim_time", false);

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
	//// sample object motions (new)
	/////////////////////////////////////////////
	fixture.particleFilter->computeAndApplyActionModel(fixture.motionData, fixture.sparseTracker, fixture.denseTracker);
	for (int i = 0; i<fixture.particleFilter->numParticles; ++i)
	{
		std_msgs::Header header; header.frame_id = scenario->mapServer->getWorldFrame(); header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*fixture.particleFilter->particles[i].state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "New state particle " << i << " shown!" << std::endl;
		sleep(10);
		{
			DataLog logger(logDir + expDirName + "particle_" + std::to_string(1) + "_" + std::to_string(i) + ".bag");
			logger.activeChannels.insert("particle");
			logger.log<OccupancyData>("particle", *fixture.particleFilter->particles[i].state);
			ROS_INFO_STREAM("Logged new particle " << i);
		}
	}

	return 0;
}