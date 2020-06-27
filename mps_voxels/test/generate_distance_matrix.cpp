//
// Created by pricear on 2020-06-26.
//

#include <mps_voxels/VoxelRegion.h>
#include <mps_voxels/JaccardMatch.h>
#include <mps_voxels/parameters/VoxelRegionBuilder.hpp>
#include <mps_voxels/OccupancyData.h>
#include <mps_voxels/SensorHistorian.h>
#include <mps_voxels/util/package_paths.h>
#include <mps_voxels/logging/DataLog.h>
#include <mps_voxels/logging/log_occupancy_data.h>
#include <mps_voxels/visualization/visualize_occupancy.h>
#include <mps_voxels/visualization/dispersed_colormap.h>

#include <regex>
#include <boost/filesystem.hpp>

int main(int argc, char* argv[])
{
//	ros::init(argc, argv, "generate_distance_matrix");
//	ros::NodeHandle nh, pnh("~");

	const std::string testDirName = "package://mps_test_data/";
	const std::string expDirName = "2020-06-24T21:04:51.344795/";
	const std::string logDir = mps::parsePackageURL(testDirName) + "/" + expDirName;
	const std::string mixDir = logDir + "/result_particles/mixture/";
	const std::regex my_filter("particleMixed_(.+)_(.+)_(.+)\\.bag");

	int numGenerations = 0;
	int numParticles = 0;
	int numMerges = 0;
	boost::filesystem::directory_iterator end_itr; // Default ctor yields past-the-end
	for (boost::filesystem::directory_iterator i(mixDir); i != end_itr; ++i)
	{
		// Skip if not a file
		if (!boost::filesystem::is_regular_file(i->status())) continue;

		std::smatch what;

		// Skip if no match:
		if (!std::regex_match(i->path().filename().string(), what, my_filter)) continue;

		numGenerations = std::max(numGenerations, std::stoi(what[1]) + 1);
		numParticles = std::max(numParticles, std::stoi(what[2]) + 1);
		numMerges = std::max(numParticles, std::stoi(what[3]) + 1);
	}

	std::vector<std::string> names;
	for (int g = 0; g < numGenerations; ++g)
	{
		for (int p = 0; p < numParticles; ++p)
		{
			for (int m = 0; m < numMerges; ++m)
			{
				names.emplace_back("particleMixed_" + std::to_string(g) + "_" + std::to_string(p) + "_" + std::to_string(m) + ".bag");
			}
		}
	}

	YAML::Node rosparams = YAML::LoadFile(logDir + "rosparam.yaml");

	std::shared_ptr<mps::VoxelRegion> region = std::make_shared<mps::VoxelRegion>(
		mps::VoxelRegionBuilder::build(rosparams["scene_explorer"]["roi"]));
	const std::string globalFrame = region->frame_id;

	Eigen::MatrixXd D(names.size(), names.size());
	for (size_t i = 0; i < names.size(); ++i)
	{
		std::cout << i << std::endl;
		mps::DataLog particleLogA(mixDir + names[i], {"particle"}, rosbag::BagMode::Read);
		mps::OccupancyData a = particleLogA.load<mps::OccupancyData>("particle");

		if (a.vertexState.size() != a.voxelRegion->num_vertices()) { throw std::logic_error("Fake news (a)!"); }

		for (size_t j = i+1; j < names.size(); ++j)
		{
			mps::DataLog particleLogB(mixDir + names[j], {"particle"}, rosbag::BagMode::Read);
			mps::OccupancyData b = particleLogB.load<mps::OccupancyData>("particle");

			int numObjects = std::max(a.objects.size(), b.objects.size());
			mps::JaccardMatch3D J(a, b);
			D(i, j) = 1.0 - (J.match.first/(double)numObjects);
		}
	}


	std::fstream out(logDir + "distances.csv", std::ios::out);
	for (size_t i = 0; i < names.size(); ++i)
	{
		for (size_t j = 0; j < names.size(); ++j)
		{
			double dist = 0.0;
			if (i < j)
				dist = D(i, j);
			else
				dist = D(j, i);

			out << dist;
			if (j < names.size() - 1)
				out << ",";
		}
		out << std::endl;
	}

	return 0;
}