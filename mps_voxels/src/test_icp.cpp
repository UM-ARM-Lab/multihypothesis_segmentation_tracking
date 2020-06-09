#include <iostream>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/PointCloud2.h>

#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_occupancy_data.h"
#include "mps_voxels/logging/log_sensor_history.h"
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

class icpTestFixture
{
public:
	std::unique_ptr<ParticleFilter> particleFilter;
	std::unique_ptr<robot_model_loader::RobotModelLoader> mpLoader;
	robot_model::RobotModelPtr pModel;
	std::shared_ptr<Scenario> scenario;
	SensorHistoryBuffer motionData;
	std::unique_ptr<Tracker> sparseTracker;
	std::unique_ptr<DenseTracker> denseTracker;

	void SetUp()
	{
		ros::NodeHandle nh, pnh("~");

		std::string logDir = parsePackageURL(testDirName);

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
		denseTracker = std::make_unique<SiamTracker>();

		double resolution = 0.010;
		pnh.getParam("resolution", resolution);
		particleFilter = std::make_unique<ParticleFilter>(scenario, resolution,
		                                                  scenario->minExtent.head<3>(),
		                                                  scenario->maxExtent.head<3>(), 1);

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
		Particle inputParticle;
		inputParticle.state = std::make_shared<Particle::ParticleData>(particleFilter->voxelRegion);
		DataLog loader(logDir + expDirName + "particle_0_" + std::to_string(0) + ".bag", {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("particle");
		*inputParticle.state = loader.load<OccupancyData>("particle");
		particleFilter->particles.push_back(inputParticle);
		particleFilter->particles[0].state->uniqueObjectLabels = getUniqueObjectLabels(particleFilter->particles[0].state->vertexState);
		std::cerr << "Successfully loaded." << std::endl;

	}
};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_icp");
	ros::NodeHandle nh, pnh("~");

	icpTestFixture fixture;
	fixture.SetUp();

	ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
	std::default_random_engine rng;

	std::string logDir = parsePackageURL(testDirName);

	/////////////////////////////////////////////
	//// sample object motions
	/////////////////////////////////////////////
	std::map<ObjectIndex, RigidTF> labelToMotionLookup;

	moveit::Pose worldTcamera;
	const auto& cameraModel = fixture.motionData.cameraModel;
	{
		const ros::Time queryTime = ros::Time(0); // fixture.motionData.rgb.begin()->first;
		const ros::Duration timeout = ros::Duration(5.0);
		std::string tfError;
		bool canTransform = fixture.motionData.tfs->canTransform(fixture.scenario->worldFrame, cameraModel.tfFrame(), queryTime, &tfError);

		if (!canTransform) // ros::Duration(5.0)
		{
			ROS_ERROR_STREAM("Failed to look up transform between '" << fixture.scenario->worldFrame << "' and '"
			                                                         << cameraModel.tfFrame() << "' with error '"
			                                                         << tfError << "'.");
			throw std::runtime_error("Sadness.");
		}

		tf::StampedTransform cameraFrameInTableCoordinates;
		const auto temp = fixture.motionData.tfs->lookupTransform(cameraModel.tfFrame(), fixture.scenario->worldFrame, queryTime);
		tf::transformStampedMsgToTF(temp, cameraFrameInTableCoordinates);
		tf::transformTFToEigen(cameraFrameInTableCoordinates.inverse(), worldTcamera);
	}

	// Image region of objects currently
	const cv::Rect objectsROI = occupancyToROI(*fixture.particleFilter->particles[0].state, cameraModel, worldTcamera);
	cv::Mat segParticle = rayCastOccupancy(*fixture.particleFilter->particles[0].state, cameraModel, worldTcamera, objectsROI);

	std::map<uint16_t, mps_msgs::AABBox2d> labelToBBoxLookup = getBBox(segParticle, objectsROI, 5);
	std::cerr << "number of bounding boxes in segParticle: " << labelToBBoxLookup.size() << std::endl;

	for (auto& pair : labelToBBoxLookup)
	{
		if (pair.first != 19 && pair.first != 23 && pair.first != 24) continue;
		std::cout << "-------------------------------------------------------------------------------------" << std::endl;
		std::cout << "Current label is " << pair.first << std::endl;
		std::shared_ptr<const ObjectActionModel> oam = estimateMotion(fixture.scenario, fixture.motionData,
			segParticle, pair.first, pair.second, fixture.sparseTracker, fixture.denseTracker, 1);
	}

	return 0;
}