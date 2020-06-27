//
// Created by kunhuang on 6/26/20.
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
const std::string productDir = "productParticles/";
const int numIter = 4; /// # buffer bags
const int numParticles = 25;

class ParticleFilterTestFixture
{
public:
	std::unique_ptr<ParticleFilter> particleFilter;
	std::unique_ptr<robot_model_loader::RobotModelLoader> mpLoader;
	robot_model::RobotModelPtr pModel;
	std::shared_ptr<Scenario> scenario;

	void SetUp()
	{
		ros::NodeHandle nh, pnh("~");

		// Load robot and scenario configurations
		mpLoader = std::make_unique<robot_model_loader::RobotModelLoader>();
		pModel = mpLoader->getModel();
		scenario = scenarioFactory(nh, pnh, pModel);
		scenario->segmentationClient = std::make_shared<RGBDSegmenter>(nh);
		scenario->completionClient = std::make_shared<VoxelCompleter>(nh);


		double resolution = 0.010;
		pnh.getParam("resolution", resolution);
		particleFilter = std::make_unique<ParticleFilter>(scenario, resolution,
		                                                  scenario->minExtent.head<3>(),
		                                                  scenario->maxExtent.head<3>(), numParticles);
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


	/////////////////////////////////////////////
	//// Load Buffer
	/////////////////////////////////////////////
	for (int i = 0; i<numIter; ++i)
	{
		DataLog loader(logDir + expDirName + "buffer_" + std::to_string(i) + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("buffer");
		SensorHistoryBuffer motionData = loader.load<SensorHistoryBuffer>("buffer");
		std::cerr << "Successfully loaded buffer " << i << std::endl;
		std::cerr << "number of frames: " << motionData.rgb.size() << std::endl;

		ros::Time initialTime = motionData.rgb.begin()->first;
		std::shared_ptr<Scene> initialScene = computeSceneFromSensorHistorian(scenario, motionData, initialTime, scenario->worldFrame);

		fixture.particleFilter->particles.clear();
		fixture.particleFilter->initializeParticles(initialScene);

		for (int j = 0; j<fixture.particleFilter->numParticles; ++j)
		{
			DataLog logger(logDir + expDirName + productDir + "particle_" + std::to_string(i) + "_" + std::to_string(j) + ".bag");
			logger.activeChannels.insert("particle");
			logger.log<OccupancyData>("particle", *fixture.particleFilter->particles[i].state);
			logger.activeChannels.insert("weight");
//			logger.log<> weight;
			ROS_INFO_STREAM("Logged X_t' " << i);
		}
	}


	return 0;
}