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
#include <mps_voxels/logging/DataLog.h>
#include <mps_voxels/logging/log_occupancy_data.h>
#include <mps_voxels/image_utils.h>

#include <mps_simulation/GazeboModel.h>
#include <mps_simulation/GazeboModelState.h>


using VoxelColormap = std::map<mps::VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>;



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
	VoxelColormap cmapA, cmapB;

	Metrics(const OccupancyData& hypothesis, const OccupancyData& truth, std::default_random_engine& rng)
		: match(hypothesis, truth)
	{
		for (const auto& m : match.match.second)
		{
			std::cerr << m.left << "\t<->\t" << m.right << std::endl;

			std_msgs::ColorRGBA color = randomColorMsg(rng);

			cmapA.emplace(m.left, color);
			cmapB.emplace(m.right, color);
		}

		extend(cmapA, hypothesis.vertexState, rng);
		extend(cmapB, truth.vertexState, rng);
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


	const std::string ground_truth = "/tmp/gazebo_segmentation/gt_occupancy.bag";
	const std::string globalFrame = "table_surface";

	std::shared_ptr<mps::VoxelRegion> region = std::make_shared<mps::VoxelRegion>(mps::VoxelRegionBuilder::build(YAML::Load("{roi: {min: {x: -0.4, y: -0.6, z: -0.020}, max: {x: 0.4, y: 0.6, z: 0.5}, resolution: 0.01}}")));

	mps::OccupancyData b(region);
	mps::DataLog loaderB(ground_truth, {"gt"}, rosbag::BagMode::Read);
	loaderB.load<mps::OccupancyData>("gt", b);
	b.voxelRegion = region;
	if (b.vertexState.size() != b.voxelRegion->num_vertices()) { throw std::logic_error("Fake news (b)!"); }


	std::random_device rd;
	int seed = rd(); //0;
	std::default_random_engine rng = std::default_random_engine(seed);

	ros::Publisher visualPubA = nh.advertise<visualization_msgs::MarkerArray>("visualization_a", 1, true);
	ros::Publisher visualPubB = nh.advertise<visualization_msgs::MarkerArray>("visualization_b", 1, true);

	while(ros::ok())
	{
		for (int p = 0; p < 5; ++p)
		{
			const std::string generated = "/tmp/scene_explorer_f1ee65a8-b378-4d9a-baee-08744061366f/particle_" + std::to_string(p) + "_experiment_world_02_25.bag";

			mps::OccupancyData a(region);
			mps::DataLog loaderA(generated, {"particle"}, rosbag::BagMode::Read);
			loaderA.load<mps::OccupancyData>("particle", a);
			a.voxelRegion = region;
			if (a.vertexState.size() != a.voxelRegion->num_vertices()) { throw std::logic_error("Fake news (a)!"); }

			mps::Metrics metrics(a, b, rng);

			int trueNeg = 0;
			int truePos = 0;
			int other = 0;
			for (size_t i = 0; i < b.voxelRegion->num_vertices(); ++i)
			{
				if (a.vertexState[i] == mps::VoxelRegion::FREE_SPACE && b.vertexState[i] == mps::VoxelRegion::FREE_SPACE)
				{
					++trueNeg;
				}
				else if (a.vertexState[i] != mps::VoxelRegion::FREE_SPACE && b.vertexState[i] != mps::VoxelRegion::FREE_SPACE)
				{
					++truePos;
				}
				else
				{
					++other;
				}
			}

			std::cerr << p << std::endl;
			std::cerr << "\t" << metrics.match.match.first << std::endl;
			std::cerr << "\t" << trueNeg << "\t" << truePos << "\t" << other << std::endl;
			std::cerr << "\t" << trueNeg/(double)region->num_vertices() << "\t" << truePos/(double)region->num_vertices() << "\t" << other/(double)region->num_vertices() << std::endl;

			std_msgs::Header header;
			header.frame_id = globalFrame;
			header.stamp = ros::Time::now();
			auto markers = mps::visualize(a, header, metrics.cmapA);
			visualPubA.publish(markers);
			markers = mps::visualize(b, header, metrics.cmapB);
			visualPubB.publish(markers);
		}
	}

	return 0;
}