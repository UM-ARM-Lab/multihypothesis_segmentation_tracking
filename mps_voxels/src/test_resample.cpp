//
// Created by kunhuang on 6/26/20.
//

#include <iostream>
#include <ros/ros.h>

#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_sensor_history.h"

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

using namespace mps;
using VoxelColormap = std::map<mps::VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>;

const std::string testDirName = "package://mps_test_data/";
const std::string expDirName = "2020-06-24T21:04:51.344795/";
const std::string logDir = parsePackageURL(testDirName);
const std::string resultDir = "result_particles/";


int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_resample");
	ros::NodeHandle nh, pnh("~");

	ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
	std::mt19937 rng;

	for (int newID=0; newID<5; ++newID)
	{
		for (int oldID=0; oldID<5; ++oldID)
		{
			for (int iter=0; iter<5; ++iter)
			{
				Particle p;
				Eigen::Vector3d rmin(-1, -1, -1);
				Eigen::Vector3d rmax(1, 1, 1);
				std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);
				p.state = std::make_shared<Particle::ParticleData>(v);
				DataLog loader(logDir + expDirName + resultDir + "mixture/" + "particleMixed_" +
				               std::to_string(newID) + "_" + std::to_string(oldID) + "_" + std::to_string(iter) + ".bag",
				               {}, rosbag::bagmode::Read);
				loader.activeChannels.insert("particle");
				*p.state = loader.load<OccupancyData>("particle");
				ROS_INFO_STREAM("Loaded mixed particle " << newID << " " << oldID);

				std_msgs::Header header; header.frame_id = "table_surface"; header.stamp = ros::Time::now();
				auto pfMarkers = mps::visualize(*p.state, header, rng);
				visualPub.publish(pfMarkers);
				std::cerr << "mixed particle " << newID << " " << oldID << std::endl;
				usleep(500000);

			}
		}
	}


	return 0;
}