//
// Created by pricear on 2020-05-29.
//

#include <mps_voxels/VoxelRegion.h>
#include <mps_voxels/JaccardMatch.h>
#include <mps_voxels/parameters/VoxelRegionBuilder.hpp>
#include <mps_voxels/visualization/visualize_occupancy.h>
#include <mps_voxels/visualization/visualize_voxel_region.h>
#include <mps_voxels/OccupancyData.h>
#include <mps_voxels/logging/DataLog.h>
#include <mps_voxels/logging/log_occupancy_data.h>

using Colormap = std::map<mps::VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>;

std_msgs::ColorRGBA randomColor(std::default_random_engine& rng)
{
	std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));

	std_msgs::ColorRGBA color;
	color.r = uni(rng);
	color.g = uni(rng);
	color.b = uni(rng);
	color.a = 1.0;
	return color;
}

void extend(Colormap& cmap, const mps::VoxelRegion::VertexLabels& labels, std::default_random_engine& rng)
{
	for (int label : labels)
	{
		if (label == mps::VoxelRegion::FREE_SPACE) { continue; }
		if (cmap.find(label) == cmap.end())
		{
			cmap.insert({label, randomColor(rng)});
		}
	}
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

			mps::JaccardMatch3D match(a, b);

			Colormap cmapA, cmapB;

			for (const auto& m : match.match.second)
			{
				std::cerr << m.left << "\t<->\t" << m.right << std::endl;

				std_msgs::ColorRGBA color = randomColor(rng);

				cmapA.insert({m.left, color});
				cmapB.insert({m.right, color});
			}

			extend(cmapA, a.vertexState, rng);
			extend(cmapB, b.vertexState, rng);

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
			std::cerr << "\t" << match.match.first << std::endl;
			std::cerr << "\t" << trueNeg << "\t" << truePos << "\t" << other << std::endl;
			std::cerr << "\t" << trueNeg/(double)region->num_vertices() << "\t" << truePos/(double)region->num_vertices() << "\t" << other/(double)region->num_vertices() << std::endl;

			std_msgs::Header header;
			header.frame_id = globalFrame;
			header.stamp = ros::Time::now();
			auto markers = mps::visualize(a, header, cmapA);
			visualPubA.publish(markers);
			markers = mps::visualize(b, header, cmapB);
			visualPubB.publish(markers);
		}
	}

	return 0;
}