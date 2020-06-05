/*
 * Copyright (c) 2020 Andrew Price
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <mps_voxels/VoxelRegion.h>
#include <mps_voxels/JaccardMatch.h>
#include <mps_voxels/parameters/VoxelRegionBuilder.hpp>
#include <mps_voxels/visualization/visualize_occupancy.h>
#include <mps_voxels/visualization/visualize_voxel_region.h>
#include <mps_voxels/OccupancyData.h>
#include <mps_voxels/SensorHistorian.h>
#include <mps_voxels/logging/DataLog.h>
#include <mps_voxels/logging/log_occupancy_data.h>
#include <mps_voxels/logging/log_sensor_history.h>
#include <mps_voxels/image_utils.h>

#include <mps_simulation/GazeboModel.h>
#include <mps_simulation/GazeboModelState.h>
#include <mps_simulation/paths.h>
#include <mps_simulation/loading.h>
#include <mps_test/CPUPixelizer.h>
#include <mps_test/ROSVoxelizer.h>
#include <tf_conversions/tf_eigen.h>

using VoxelColormap = std::map<mps::VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>;

using namespace mps;

std_msgs::ColorRGBA randomColorMsg(std::default_random_engine& rng)
{
	std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));

	std_msgs::ColorRGBA color;
	color.r = uni(rng);
	color.g = uni(rng);
	color.b = uni(rng);
	color.a = 1.0;
	return color;
}

void extend(VoxelColormap& cmap, const mps::VoxelRegion::VertexLabels& labels, std::default_random_engine& rng)
{
	for (int label : labels)
	{
		if (label == mps::VoxelRegion::FREE_SPACE) { continue; }
		if (cmap.find(label) == cmap.end())
		{
			cmap.emplace(label, randomColorMsg(rng));
		}
	}
}

namespace mps
{

struct Metrics
{
	mps::JaccardMatch3D match;
	VoxelColormap cmapA;

	Metrics(const OccupancyData& hypothesis, const OccupancyData& truth, const VoxelColormap& cmapGT, std::default_random_engine& rng)
		: match(hypothesis, truth)
	{
		for (const auto& m : match.match.second)
		{
			cmapA.emplace(m.left, cmapGT.at(m.right));
//			cmapB.emplace(m.right, color);
		}

		extend(cmapA, hypothesis.vertexState, rng);
//		extend(cmapB, truth.vertexState, rng);
	}
};
/*
class SegmentationEvaluator
{
public:
	std::vector<std::shared_ptr<GazeboModel>> models;

	Metrics evaluate(const OccupancyData& occupancy, const std::vector<GazeboModelState>& states);
};
*/
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "test_occupancy_comparison");
	ros::NodeHandle nh, pnh("~");

	const std::string workingDir = "/tmp/scene_explorer/2020-06-05T16:12:56.229850/";
//	const std::string workingDir = "/tmp/scene_explorer/2020-06-05T16:24:42.273504/";
//	const std::string ground_truth = "/tmp/gazebo_segmentation/gt_occupancy.bag";
	const std::string globalFrame = "table_surface";

	std::shared_ptr<mps::VoxelRegion> region = std::make_shared<mps::VoxelRegion>(mps::VoxelRegionBuilder::build(YAML::Load("{roi: {min: {x: -0.4, y: -0.6, z: -0.020}, max: {x: 0.4, y: 0.6, z: 0.5}, resolution: 0.01}}")));

	std::random_device rd;
	int seed = rd(); //0;
	std::default_random_engine rng = std::default_random_engine(seed);

	// TODO: Load from directory
	const int numGenerations = 2;
	const int numParticles = 2;

	ros::Publisher particlePubGT = nh.advertise<visualization_msgs::MarkerArray>("visualization_gt", 1, true);
	std::vector<std::shared_ptr<ros::Publisher>> particlePubs;
	for (int p = 0; p < numParticles; ++p)
	{
		particlePubs.emplace_back(std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("visualization_" + std::to_string(p), 1, true)));
	}

	//-------------------------------------------------------------------------
	// Ground Truth
	//-------------------------------------------------------------------------
	mps::SensorHistoryBuffer motionData;
	mps::DataLog loaderB(workingDir + "buffer_0.bag", {"buffer"}, rosbag::BagMode::Read);
	loaderB.load<mps::SensorHistoryBuffer>("buffer", motionData);

	std::string worldFileParam;
	if (!pnh.getParam("/simulation_world", worldFileParam))
	{
		ROS_FATAL_STREAM("Unable to read world file name from parameter server '" << worldFileParam << "'");
		return -1;
	}
	// Load Gazebo world file and get meshes
	const std::string gazeboWorldFilename = parsePackageURL(worldFileParam);

	std::vector<std::string> modelsToIgnore;
	pnh.param("ignore", modelsToIgnore, std::vector<std::string>());
	std::set<std::string> modsToIgnore(modelsToIgnore.begin(), modelsToIgnore.end());
	if (!modelsToIgnore.empty())
	{
		std::cerr << "Ignoring models:" << std::endl;
		for (const auto& m : modelsToIgnore)
		{
			std::cerr << "\t" << m << std::endl;
		}
	}

	std::map<std::string, std::shared_ptr<GazeboModel>> shapeModels = mps::getWorldFileModels(gazeboWorldFilename, modsToIgnore);
	if (shapeModels.empty())
	{
		ROS_WARN_STREAM("No models were loaded from world file '" << gazeboWorldFilename << "'.");
	}
	for (auto& shape : shapeModels)
	{
		shape.second->computeBoundingSphere();
	}

	std::vector<std::shared_ptr<GazeboModel>> tmpModels; tmpModels.reserve(shapeModels.size());
	for (const auto& m : shapeModels) { tmpModels.push_back(m.second); }

	std::unique_ptr<SceneVoxelizer> voxelizer = std::make_unique<ROSVoxelizer>(tmpModels);


	// Create colormap for ground truth
	VoxelColormap cmapGT;
	for (VoxelColormap::key_type i = 0; i < static_cast<VoxelColormap::key_type>(shapeModels.size()); ++i)
	{
		cmapGT.emplace(i+1, randomColorMsg(rng));
	}

//	b.voxelRegion = region;
//	if (b.vertexState.size() != b.voxelRegion->num_vertices()) { throw std::logic_error("Fake news (b)!"); }

	while(ros::ok())
	{
		for (int generation = 0; generation < numGenerations; ++generation)
		{
			// Publish images here
			// Compute poses from buffer
			std::vector<Eigen::Isometry3d> poses;
			for (const auto& pair : shapeModels)
			{
				// TODO: How to get time for particle generation?
				const std::string objectFrame = "mocap_" + pair.first;
				const ros::Time queryTime = (generation == 0) ? motionData.rgb.begin()->second->header.stamp + ros::Duration(15.0) : ros::Time(0); // buffer.rgb.begin()->first;
				std::string tfError;
				bool canTransform = motionData.tfs->canTransform(globalFrame, objectFrame, queryTime, &tfError);

				if (!canTransform) // ros::Duration(5.0)
				{
					ROS_ERROR_STREAM("Failed to look up transform between '" << globalFrame << "' and '"
					                                                         << objectFrame << "' with error '"
					                                                         << tfError << "'.");
					throw std::runtime_error("Sadness.");
				}

				tf::StampedTransform objectFrameInTableCoordinates;
				const auto temp = motionData.tfs->lookupTransform(objectFrame, globalFrame, queryTime);
				tf::transformStampedMsgToTF(temp, objectFrameInTableCoordinates);
				Eigen::Isometry3d pose;
				tf::transformTFToEigen(objectFrameInTableCoordinates.inverse(), pose);
				poses.push_back(pose);
			}

			mps::VoxelRegion::VertexLabels labels = voxelizer->voxelize(*region, poses);
			mps::OccupancyData b(region, labels);

			std_msgs::Header header;
			header.frame_id = globalFrame;
			header.stamp = ros::Time::now();
			particlePubGT.publish(mps::visualize(b, header, cmapGT));


			for (int p = 0; p < numParticles; ++p)
			{
				const std::string generated = workingDir + "particle_"
				                              + std::to_string(generation) + "_"
				                              + std::to_string(p) + ".bag";

				mps::OccupancyData a(region);
				mps::DataLog loaderA(generated, {"particle"}, rosbag::BagMode::Read);
				loaderA.load<mps::OccupancyData>("particle", a);
				a.voxelRegion = region;
				if (a.vertexState.size() != a.voxelRegion->num_vertices()) { throw std::logic_error("Fake news (a)!"); }

				mps::Metrics metrics(a, b, cmapGT, rng);

				int trueNeg = 0;
				int truePos = 0;
				int other = 0;
				for (size_t i = 0; i < b.voxelRegion->num_vertices(); ++i)
				{
					if (a.vertexState[i] == mps::VoxelRegion::FREE_SPACE
					    && b.vertexState[i] == mps::VoxelRegion::FREE_SPACE)
					{
						++trueNeg;
					}
					else if (a.vertexState[i] != mps::VoxelRegion::FREE_SPACE
					         && b.vertexState[i] != mps::VoxelRegion::FREE_SPACE)
					{
						++truePos;
					}
					else
					{
						++other;
					}
				}

				std::cerr << generation << ": " << p << std::endl;
				std::cerr << "\t" << metrics.match.match.first << std::endl;
				std::cerr << "\t" << trueNeg << "\t" << truePos << "\t" << other << std::endl;
				std::cerr << "\t" << trueNeg / (double) region->num_vertices() << "\t"
				          << truePos / (double) region->num_vertices() << "\t"
				          << other / (double) region->num_vertices() << std::endl;

				header.stamp = ros::Time::now();
				particlePubs[p]->publish(mps::visualize(a, header, metrics.cmapA));
				particlePubGT.publish(mps::visualize(b, header, cmapGT));
			}
		}
	}

	return 0;
}