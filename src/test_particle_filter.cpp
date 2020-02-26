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
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/ParticleFilter.h"
#include "mps_voxels/visualization/visualize_occupancy.h"
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/util/package_paths.h"

#include <moveit/robot_model_loader/robot_model_loader.h>

using namespace mps;

const std::string testDirName = "package://mps_test_data/";
const std::string worldname = "experiment_world_02_21";


class ParticleFilterTestFixture
{
public:
	std::unique_ptr<robot_model_loader::RobotModelLoader> mpLoader;
	robot_model::RobotModelPtr pModel;
	std::shared_ptr<Scenario> scenario;
	std::shared_ptr<Scene> initialScene;
	SensorHistoryBuffer motionData;

	void SetUp()
	{
		ros::NodeHandle nh, pnh("~");

		// Load robot and scenario configurations
		mpLoader = std::make_unique<robot_model_loader::RobotModelLoader>();
		pModel = mpLoader->getModel();
		scenario = scenarioFactory(nh, pnh, pModel);
		scenario->segmentationClient = std::make_shared<RGBDSegmenter>(nh);

		/////////////////////////////////////////////
		//// Load initial sensor data
		/////////////////////////////////////////////
		std::string logDir = parsePackageURL(testDirName);
		{
			DataLog loader(logDir + "explorer_buffer_" + worldname + ".bag", {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("buffer");
			loader.load<SensorHistoryBuffer>("buffer", motionData);
			std::cerr << "Successfully loaded." << std::endl;
		}
		std::cerr << "number of frames: " << motionData.rgb.size() << std::endl;

		ros::Time initialTime = motionData.rgb.begin()->first;
		initialScene = std::make_shared<Scene>(computeSceneFromSensorHistorian(scenario, motionData, initialTime, scenario->worldFrame));


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


	std::string logDir = parsePackageURL(testDirName);

	/////////////////////////////////////////////
	//// Initialize Utils
	/////////////////////////////////////////////

	std::unique_ptr<Tracker> sparseTracker = std::make_unique<Tracker>();
	sparseTracker->track_options.featureRadius = 200.0f;
	sparseTracker->track_options.pixelRadius = 1000.0f;
	sparseTracker->track_options.meterRadius = 1.0f;
	std::unique_ptr<DenseTracker> denseTracker = std::make_unique<SiamTracker>();

	std::shared_ptr<Scenario> scenario = fixture.scenario;
	scenario->mapServer = std::make_shared<LocalOctreeServer>(pnh);

	octomap::point3d tMin(scenario->minExtent.x(), scenario->minExtent.y(), scenario->minExtent.z());
	octomap::point3d tMax(scenario->maxExtent.x(), scenario->maxExtent.y(), scenario->maxExtent.z());
	scenario->mapServer->m_octree->setBBXMin(tMin);
	scenario->mapServer->m_octree->setBBXMax(tMax);

	/////////////////////////////////////////////
	//// Load Original Particle
	/////////////////////////////////////////////
	Particle particle;
	//// in case the voxelRegion is not initialized, the value of this voxelRegion doesn't matter
	const double resolution = 0.010;
	mps::VoxelRegion::vertex_descriptor dims = roiToVoxelRegion(resolution,
	                                                            scenario->minExtent.head<3>().cast<double>(),
	                                                            scenario->maxExtent.head<3>().cast<double>());
	std::shared_ptr<VoxelRegion> voxelRegion = std::make_shared<VoxelRegion>(dims, resolution,
	                                                                         scenario->minExtent.head<3>().cast<double>(),
	                                                                         scenario->maxExtent.head<3>().cast<double>());

	particle.state = std::make_shared<OccupancyData>(voxelRegion);
	{
		DataLog loader(logDir + "/explorer_particle_state_" + worldname + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("particleState");
		loader.load<OccupancyData>("particleState", *particle.state);
		std::cerr << "Successfully loaded." << std::endl;
	}
	std::cerr << "Voxel state size = " << particle.state->vertexState.size() << std::endl;
	particle.state->uniqueObjectLabels = getUniqueObjectLabels(particle.state->vertexState);

	// Visualize state
	std_msgs::Header header; header.frame_id = "table_surface"; header.stamp = ros::Time::now();
	auto pfMarkers = mps::visualize(*particle.state, header, rng);
	visualPub.publish(pfMarkers);
	std::cerr << "State particle shown!" << std::endl;
	sleep(2);

	/////////////////////////////////////////////
	//// Visualization of first point cloud
	/////////////////////////////////////////////
	pcl::PointCloud<PointT>::Ptr firstPC = fixture.initialScene->cropped_cloud;//imagesToCloud(fixture.motionData.rgb.begin()->second->image, fixture.motionData.depth.begin()->second->image, fixture.motionData.cameraModel);

	ros::Publisher pcPub1 = pnh.advertise<pcl::PointCloud<PointT>>("firstPC", 1, true);

	firstPC->header.frame_id = fixture.motionData.cameraModel.tfFrame();
	ros::Rate loop_rate(4);
	while (nh.ok())
	{
		pcl_conversions::toPCL(ros::Time::now(), firstPC->header.stamp);
		pcPub1.publish(*firstPC);
		ros::spinOnce();
//		loop_rate.sleep();
	}
	std::cerr << "First frame pointcloud shown" << std::endl;

	/////////////////////////////////////////////
	//// Free space refinement
	/////////////////////////////////////////////
	scenario->mapServer->insertCloud(firstPC, fixture.initialScene->worldTcamera);
	octomap::OcTree* sceneOctree = scenario->mapServer->getOctree();

	refineParticleFreeSpace(particle, sceneOctree);
	pfMarkers = mps::visualize(*particle.state, header, rng);
	visualPub.publish(pfMarkers);
	std::cerr << "Refined state particle shown!" << std::endl;
	sleep(5);

	/////////////////////////////////////////////
	//// sample object motions (new)
	/////////////////////////////////////////////
	std::unique_ptr<ParticleFilter> particleFilter = std::make_unique<ParticleFilter>(scenario, dims, resolution,
	                                                                                  scenario->minExtent.head<3>().cast<double>(),
	                                                                                  scenario->maxExtent.head<3>().cast<double>(), 1);
	particleFilter->voxelRegion = particle.state->voxelRegion;

	// TODO: Initialize particle filter

	// TODO: Apply history/action to particles
//	Particle outputParticle = particleFilter->applyActionModel(particle, fixture.motionData.cameraModel, worldTcamera,
//	                                                           fixture.motionData, sparseTracker, denseTracker,10);
//	auto pfnewmarker = mps::visualize(*outputParticle.state, header, rng);
//	visualPub.publish(pfnewmarker);
//	std::cerr << "Predicted state particle shown!" << std::endl;
//	sleep(5);

	// TODO: Apply scene #2 measurement to particles

	/////////////////////////////////////////////
	//// Free space refinement
	/////////////////////////////////////////////
//	pcl::PointCloud<PointT>::Ptr finalPC = imagesToCloud(fixture.motionData.rgb.rbegin()->second->image, fixture.motionData.depth.rbegin()->second->image, fixture.motionData.cameraModel);
//	scenario->mapServer->insertCloud(finalPC, worldTcamera);
//	sceneOctree = scenario->mapServer->getOctree();
//
//	refineParticleFreeSpace(outputParticle, sceneOctree);
//	pfnewmarker = mps::visualize(*outputParticle.state, header, rng);
//	visualPub.publish(pfnewmarker);
//	std::cerr << "Refined predicted state particle shown!" << std::endl;
//	sleep(5);

	return 0;
}