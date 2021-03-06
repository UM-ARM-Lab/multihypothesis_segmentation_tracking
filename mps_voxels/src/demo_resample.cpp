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
#include "mps_msgs/SegmentGraph.h"

using namespace mps;
using VoxelColormap = std::map<mps::VoxelRegion::VertexLabels::value_type, std_msgs::ColorRGBA>;

const std::string testDirName = "package://mps_test_data/";
const std::string expDirName = "2020-06-30/";
const std::string logDir = parsePackageURL(testDirName);
const int generation = 1;
const ExperimentDir checkpointDir;
const int numParticles = 5;
const bool isComputingWeight = false;

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

		Tracker::TrackingOptions opts;
		opts.directory = logDir + expDirName;

		double resolution = 0.010;
		pnh.getParam("resolution", resolution);
		particleFilter = std::make_unique<ParticleFilter>(scenario, resolution,
														  scenario->minExtent.head<3>(),
														  scenario->maxExtent.head<3>(), numParticles);
	}
};

std::pair<double, double>
applyMeasurementModel(const Particle &particle, const std::shared_ptr<const ParticleFilter::MeasurementSensorData> &newScene, std::mt19937 &rng)
{
	const size_t nSamples = 10;
	SegmentationTreeSampler treeSampler(newScene->segInfo);
	std::vector<std::pair<double, SegmentationCut>> segmentationSamples = treeSampler.sample(rng, nSamples, true);

	assert(newScene->roi.x >= 0);
	assert(newScene->roi.y >= 0);
	assert(newScene->roi.width > 0);
	assert(newScene->roi.height > 0);

	assert(particle.state);

	cv::Mat segParticle = rayCastOccupancy(*particle.state, newScene->cameraModel, newScene->worldTcamera, newScene->roi);
	if (segParticle.size() == cv::Size(newScene->cameraModel.cameraInfo().width, newScene->cameraModel.cameraInfo().height))
	{
		cv::Mat cropped = segParticle(newScene->roi);
		cropped.copyTo(segParticle);
	}
	MPS_ASSERT(segParticle.size() == newScene->roi.size());

	double bestMatchScore = -std::numeric_limits<double>::infinity();
	double segWeight = -std::numeric_limits<double>::infinity();

	for (size_t s = 0; s < nSamples; ++s)
	{
		const auto &segSample = segmentationSamples[s].second.segmentation.objectness_segmentation->image;
		MPS_ASSERT(newScene->roi.size() == segSample.size());
		MPS_ASSERT(segParticle.size() == segSample.size());

		mps::JaccardMatch J(segParticle, segSample);
		//		double matchScore = J.symmetricCover();
		double matchScore = J.match.first;
		if (matchScore > bestMatchScore)
		{
			bestMatchScore = matchScore;
			segWeight = segmentationSamples[s].first;
		}
	}
	std::cerr << "Best match score " << bestMatchScore << std::endl;
	std::cerr << "Relative segmentation weight " << segWeight << std::endl;
	return std::make_pair(segWeight, bestMatchScore);
}

std::vector<Particle> resampleLogWeights(const std::vector<Particle> &particles, int num, std::mt19937 &rng)
{
	std::vector<double> weightBar;

	for (auto &p : particles)
	{
		weightBar.push_back(p.weight);
	}
	double minw = *std::min_element(weightBar.begin(), weightBar.end());
	for (auto &w : weightBar)
	{
		w += minw;
		w = exp(w);
	}
	std::discrete_distribution<> distribution(weightBar.begin(), weightBar.end());

	std::vector<Particle> resampledParticles;
	for (int i = 0; i < num; ++i)
	{
		int index = distribution(rng);
		resampledParticles.push_back(particles[index]);
	}
	return resampledParticles;
}

std::pair<size_t, std::vector<int>> clusterDistances(ros::NodeHandle &nh, const Eigen::MatrixXd &D)
{
	assert(D.rows() == D.cols());
	static ros::ServiceClient segmentClient = nh.serviceClient<mps_msgs::SegmentGraph>("/segment_graph");
	if (!segmentClient.waitForExistence(ros::Duration(3)))
	{
		ROS_ERROR("Segmentation server not connected.");
		return {};
	}

	mps_msgs::SegmentGraphRequest req;

	for (int i = 0; i < D.rows(); ++i)
	{
		for (int j = 0; j < D.cols(); ++j)
		{
			req.adjacency.row_index.push_back(i);
			req.adjacency.col_index.push_back(j);
			req.adjacency.value.push_back(D(i, j));
		}
	}
	req.algorithm = "hdbscan";
	mps_msgs::SegmentGraphResponse resp;
	segmentClient.call(req, resp);

	return {resp.num_labels, resp.labels};
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "test_resample", ros::init_options::AnonymousName);
	ros::NodeHandle nh, pnh("~");

	ParticleFilterTestFixture fixture;
	fixture.SetUp();

	ros::Publisher visualPub = nh.advertise<visualization_msgs::MarkerArray>("visualization", 1, true);
	std::mt19937 rng;

	std::shared_ptr<Scene> newScene;

	std::vector<double> oldParticleQual;
	for (int i = 0; i < numParticles; ++i)
	{
		Particle p;
		Eigen::Vector3d rmin(-1, -1, -1);
		Eigen::Vector3d rmax(1, 1, 1);
		std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);
		p.state = std::make_shared<Particle::ParticleData>(v);
		if (generation == 1)
		{
			DataLog loader(logDir + expDirName + checkpointDir.checkpoints[0] + "/particle_" +
							   std::to_string(generation - 1) + "_" + std::to_string(i) + ".bag",
						   {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("particle");
			*p.state = loader.load<OccupancyData>("particle");
			std_msgs::Float64 weightMsg;
			loader.activeChannels.insert("weight");
			weightMsg = loader.load<std_msgs::Float64>("weight");

			oldParticleQual.push_back(weightMsg.data);
		}
		else
		{
			DataLog loader(logDir + expDirName + checkpointDir.checkpoints[2] + "/particle_" +
							   std::to_string(generation - 1) + "_" + std::to_string(i) + ".bag",
						   {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("particle");
			*p.state = loader.load<OccupancyData>("particle");
			std_msgs::Float64 weightMsg;
			loader.activeChannels.insert("weight");
			weightMsg = loader.load<std_msgs::Float64>("weight");

			oldParticleQual.push_back(weightMsg.data);
		}
		p.particle.id = i;
		std::cerr << "Successfully loaded old particle_" << generation - 1 << "_" << i << " with quality " << p.weight << std::endl;
	}

	std::vector<double> newMeasurementParticleQual;
	for (int i = 0; i < numParticles; ++i)
	{
		Particle p;
		Eigen::Vector3d rmin(-1, -1, -1);
		Eigen::Vector3d rmax(1, 1, 1);
		std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);
		p.state = std::make_shared<Particle::ParticleData>(v);
		DataLog loader(logDir + expDirName + checkpointDir.checkpoints[0] + "/particle_" +
						   std::to_string(generation) + "_" + std::to_string(i) + ".bag",
					   {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("particle");
		*p.state = loader.load<OccupancyData>("particle");
		p.particle.id = i;
		std_msgs::Float64 weightMsg;
		loader.activeChannels.insert("weight");
		weightMsg = loader.load<std_msgs::Float64>("weight");

		newMeasurementParticleQual.push_back(weightMsg.data);
		std::cerr << "Successfully loaded new measurement particle_" << generation << "_" << i << " with quality " << p.weight << std::endl;
	}

	std::vector<double> refinedParticleQual;
	for (int i = 0; i < numParticles; ++i)
	{
		Particle p;
		Eigen::Vector3d rmin(-1, -1, -1);
		Eigen::Vector3d rmax(1, 1, 1);
		std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);
		p.state = std::make_shared<Particle::ParticleData>(v);
		DataLog loader(logDir + expDirName + checkpointDir.checkpoints[4] + "/particle_" +
						   std::to_string(generation) + "_" + std::to_string(i) + ".bag",
					   {}, rosbag::bagmode::Read);
		loader.activeChannels.insert("particle");
		*p.state = loader.load<OccupancyData>("particle");
		p.particle.id = i;
		std_msgs::Float64 weightMsg;
		loader.activeChannels.insert("weight");
		weightMsg = loader.load<std_msgs::Float64>("weight");

		refinedParticleQual.push_back(weightMsg.data);
		std::cerr << "Successfully loaded history particle_" << generation << "_" << i << " with quality " << p.weight << std::endl;
	}

	Eigen::Vector3d rmin(-1, -1, -1);
	Eigen::Vector3d rmax(1, 1, 1);
	std::shared_ptr<VoxelRegion> v = std::make_shared<VoxelRegion>(0.01, rmin, rmax);
	std::vector<Particle> mixtureParticles;
	for (int newID = 0; newID < numParticles; ++newID)
	{
		for (int oldID = 0; oldID < numParticles; ++oldID)
		{
			for (int iter = 0; iter < 5; ++iter)
			{
				std::cout << "-------------------------------------------------------" << std::endl;
				Particle p;
				p.state = std::make_shared<Particle::ParticleData>(v);
				{
					DataLog loader(
						logDir + expDirName + checkpointDir.checkpoints[1] + "/particle_" + std::to_string(generation) +
							"_" + std::to_string(newID) + "_" + std::to_string(oldID) + "_" + std::to_string(iter) + ".bag",
						{}, rosbag::bagmode::Read);
					loader.activeChannels.insert("particle");
					*p.state = loader.load<OccupancyData>("particle");
					//				    ROS_INFO_STREAM("Loaded mixed particle " << newID << " " << oldID);
					if (!isComputingWeight)
					{
						std_msgs::Float64 weightMsg;
						loader.activeChannels.insert("weight");
						weightMsg = loader.load<std_msgs::Float64>("weight");
						p.weight = weightMsg.data;
					}
				}

				std_msgs::Header header;
				header.frame_id = "table_surface";
				header.stamp = ros::Time::now();
				auto pfMarkers = mps::visualize(*p.state, header, rng);
				visualPub.publish(pfMarkers);
				std::cerr << "mixed particle " << newID << " " << oldID << std::endl;
				//				usleep(500000);

				std::cerr << oldParticleQual[oldID] << "    " << refinedParticleQual[oldID] << "    " << newMeasurementParticleQual[newID] << std::endl;

				p.weight = oldParticleQual[oldID] + newMeasurementParticleQual[newID] + 3 * log(refinedParticleQual[oldID]);
				std::cerr << p.weight << std::endl;

				if (isComputingWeight)
				{
					//					std::pair<double, double> measurementMatchness = applyMeasurementModel(p, newScene, rng);

					DataLog logger(logDir + expDirName + checkpointDir.checkpoints[1] + "/particle_" + std::to_string(generation) +
									   "_" + std::to_string(newID) + "_" + std::to_string(oldID) + "_" + std::to_string(iter) + ".bag",
								   {}, rosbag::bagmode::Append);
					std_msgs::Float64 weightMsg;
					weightMsg.data = p.weight;
					logger.activeChannels.insert("weight");
					logger.log<std_msgs::Float64>("weight", weightMsg);
				}
				mixtureParticles.push_back(p);
			}
		}
	}

	// Cluster the mixture samples
	Eigen::MatrixXd D(mixtureParticles.size(), mixtureParticles.size());
	for (size_t i = 0; i < mixtureParticles.size(); ++i)
	{
		for (size_t j = i + 1; j < mixtureParticles.size(); ++j)
		{
			mps::JaccardMatch3D J(*mixtureParticles[i].state, *mixtureParticles[j].state);
			D(i, j) = 1.0 - (J.symmetricCover());
			D(j, i) = D(i, j);
		}
	}
	auto clusters = clusterDistances(nh, D);
	std::cerr << "Clusters: " << clusters.first << std::endl;
	std::map<int, int> histogram;
	for (const auto l : clusters.second)
	{
		histogram[l]++;
		std::cerr << l << std::endl;
	}

	std::vector<Particle> resampledParticles = resampleLogWeights(mixtureParticles, numParticles, rng);
	for (size_t p = 0; p < resampledParticles.size(); ++p)
	{
		std_msgs::Header header;
		header.frame_id = "table_surface";
		header.stamp = ros::Time::now();
		auto pfMarkers = mps::visualize(*resampledParticles[p].state, header, rng);
		visualPub.publish(pfMarkers);
		std::cerr << "resampled particle shown" << std::endl;
		usleep(500000);

		{
			DataLog logger(logDir + expDirName + ExperimentDir::checkpoints[2] + "/particle_" + std::to_string(generation) + "_" +
						   std::to_string(p) + ".bag");
			logger.activeChannels.insert("particle");
			logger.log<OccupancyData>("particle", *resampledParticles[p].state);
			std_msgs::Float64 weightMsg;
			weightMsg.data = resampledParticles[p].weight;
			logger.activeChannels.insert("weight");
			logger.log<std_msgs::Float64>("weight", weightMsg);
			ROS_INFO_STREAM("Logged estimate particle " << p);
		}
	}

	return 0;
}
