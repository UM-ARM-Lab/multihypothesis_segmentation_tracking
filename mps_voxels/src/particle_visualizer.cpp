//
// Created by kunhuang on 6/30/20.
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
const std::string expDirName = "2020-06-30/";
const std::string logDir = parsePackageURL(testDirName);
const int generation = 2;
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


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_resample", ros::init_options::AnonymousName);
	ros::NodeHandle nh, pnh("~");

//	ParticleFilterTestFixture fixture;
//	fixture.SetUp();

	ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
	std::mt19937 rng;


	Eigen::Vector3d rmin(-1, -1, -1);
	Eigen::Vector3d rmax(1, 1, 1);
	std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);

	std::vector<double> weightbar;
	size_t whichDir = 2;
	if (generation == 0) whichDir=0;
	for (int newID=0; newID<numParticles; ++newID)
	{
		std::cout << "-------------------------------------------------------" << std::endl;
		Particle p;
		p.state = std::make_shared<Particle::ParticleData>(v);
		{
			DataLog loader(
				logDir + expDirName + checkpointDir.checkpoints[whichDir] + "/particle_" + std::to_string(generation) +
				"_" + std::to_string(newID) + ".bag",
				{}, rosbag::bagmode::Read);
			loader.activeChannels.insert("particle");
			*p.state = loader.load<OccupancyData>("particle");
			std_msgs::Float64 weightMsg;
			loader.activeChannels.insert("weight");
			weightMsg = loader.load<std_msgs::Float64>("weight");
			p.weight = weightMsg.data;
			std::cerr << "weight = " << p.weight << std::endl;
			weightbar.push_back(p.weight);
		}

		std_msgs::Header header; header.frame_id = "table_surface"; header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*p.state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "mixed particle " << newID << " " << newID << std::endl;
	}

	double weightSum = 0.0;
	for (auto& w : weightbar)
	{
		weightSum+=w;
	}
	for (auto& w : weightbar)
	{
		w /= weightSum;
		std::cerr << w << std::endl;
	}

	return 0;
}