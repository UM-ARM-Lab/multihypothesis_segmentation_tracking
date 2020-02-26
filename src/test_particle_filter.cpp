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

using namespace mps;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_pf");
	ros::NodeHandle nh, pnh("~");
//	if (!nh.hasParam("/use_sim_time"))
//	{
//		ROS_INFO("No param named '/use_sim_time'");
//	}
//	nh.setParam("/use_sim_time", false);
	ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
	std::default_random_engine rng;

	/////////////////////////////////////////////
	//// Load sensor history
	/////////////////////////////////////////////
	std::string logDir = "/home/kunhuang/mps_log/0221";
	std::string worldname = "experiment_world_02_21";
	SensorHistoryBuffer buffer_out;
	{
		DataLog loader(logDir + "/explorer_buffer_" + worldname + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("buffer");
		loader.load<SensorHistoryBuffer>("buffer", buffer_out);
		std::cerr << "Successfully loaded." << std::endl;
	}
	std::cerr << "number of frames: " << buffer_out.rgb.size() << std::endl;

	/////////////////////////////////////////////
	//// Compute Scene
	/////////////////////////////////////////////
	const std::string worldFrame = "table_surface";
	computeSceneFromSensorHistorian(buffer_out, buffer_out.rgb.begin()->first, worldFrame);

	/////////////////////////////////////////////
	//// Initialize Utils
	/////////////////////////////////////////////
	Eigen::Vector3f ROImaxExtent(0.4f, 0.6f, 0.5f);
	Eigen::Vector3f ROIminExtent(-0.4f, -0.6f, -0.020f);
	octomap::point3d tMin(-0.4f, -0.6f, -0.020f);
	octomap::point3d tMax(0.4f, 0.6f, 0.5f);

	std::unique_ptr<Tracker> sparseTracker = std::make_unique<Tracker>();
	sparseTracker->track_options.featureRadius = 200.0f;
	sparseTracker->track_options.pixelRadius = 1000.0f;
	sparseTracker->track_options.meterRadius = 1.0f;
	std::unique_ptr<DenseTracker> denseTracker = std::make_unique<SiamTracker>();

	std::shared_ptr<Scenario> scenario = std::make_shared<Scenario>();
	scenario->experiment = std::make_shared<Experiment>(nh, pnh);
	scenario->mapServer = std::make_shared<LocalOctreeServer>(pnh);

	scenario->mapServer->m_octree->setBBXMin(tMin);
	scenario->mapServer->m_octree->setBBXMax(tMax);

	/////////////////////////////////////////////
	//// Load Original Particle
	/////////////////////////////////////////////
	Particle particle;
	//// in case the voxelRegion is not initialized, the value of this voxelRegion doesn't matter
	const double resolution = 0.010;
	mps::VoxelRegion::vertex_descriptor dims = roiToVoxelRegion(resolution,
	                                                            ROIminExtent.cast<double>(),
	                                                            ROImaxExtent.cast<double>());
	std::shared_ptr<VoxelRegion> voxelRegion = std::make_shared<VoxelRegion>(dims, resolution,
	                                                                         ROIminExtent.cast<double>(),
	                                                                         ROImaxExtent.cast<double>());

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
/*
	/////////////////////////////////////////////
	//// Visualization of first point cloud
	/////////////////////////////////////////////
	pcl::PointCloud<PointT>::Ptr firstPC = imagesToCloud(buffer_out.rgb.begin()->second->image, buffer_out.depth.begin()->second->image, buffer_out.cameraModel);

	ros::Publisher pcPub1 = pnh.advertise<pcl::PointCloud<PointT>>("firstPC", 1, true);

	firstPC->header.frame_id = buffer_out.cameraModel.tfFrame();
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
	scenario->mapServer->insertCloud(firstPC, worldTcamera);
	octomap::OcTree* sceneOctree = scenario->mapServer->getOctree();

	refineParticleFreeSpace(particle, sceneOctree);
	pfMarkers = mps::visualize(*particle.state, header, rng);
	visualPub.publish(pfMarkers);
	std::cerr << "Refined state particle shown!" << std::endl;
	sleep(5);
*/
	/////////////////////////////////////////////
	//// sample object motions (new)
	/////////////////////////////////////////////
	std::unique_ptr<ParticleFilter> particleFilter = std::make_unique<ParticleFilter>(scenario, dims, resolution,
	                                                                                  scenario->minExtent.head<3>().cast<double>(),
	                                                                                  scenario->maxExtent.head<3>().cast<double>(), 1);
	particleFilter->voxelRegion = particle.state->voxelRegion;

	// TODO: Initialize particle filter

	// TODO: Apply history/action to particles
//	Particle outputParticle = particleFilter->applyActionModel(particle, buffer_out.cameraModel, worldTcamera,
//	                                                           buffer_out, sparseTracker, denseTracker,10);
//	auto pfnewmarker = mps::visualize(*outputParticle.state, header, rng);
//	visualPub.publish(pfnewmarker);
//	std::cerr << "Predicted state particle shown!" << std::endl;
//	sleep(5);

	// TODO: Apply scene #2 measurement to particles

	/////////////////////////////////////////////
	//// Free space refinement
	/////////////////////////////////////////////
//	pcl::PointCloud<PointT>::Ptr finalPC = imagesToCloud(buffer_out.rgb.rbegin()->second->image, buffer_out.depth.rbegin()->second->image, buffer_out.cameraModel);
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
