//
// Created by kunhuang on 2/12/20.
//

#include <iostream>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>

#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_sensor_history.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/ParticleFilter.h"
#include "mps_voxels/visualization/visualize_occupancy.h"

using namespace mps;

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_pf");
	ros::NodeHandle nh;
	if (!nh.hasParam("/use_sim_time"))
	{
		ROS_INFO("No param named '/use_sim_time'");
	}
//	nh.setParam("/use_sim_time", false);
	ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
	std::default_random_engine rng;

	/////////////////////////////////////////////
	//// Load sensor history & particle data
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

	Particle particle;
	//// in case the voxelRegion is not initialized, the value of this voxelRegion doesn't matter
	const double resolution = 0.010;
	Eigen::Vector3f ROImaxExtent(0.4f, 0.6f, 0.5f);
	Eigen::Vector3f ROIminExtent(-0.4f, -0.6f, -0.020f);
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

	return 0;
}
