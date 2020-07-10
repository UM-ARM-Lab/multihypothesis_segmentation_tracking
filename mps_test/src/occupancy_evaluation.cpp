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
#include <mps_voxels/OccupancyData.h>
#include <mps_voxels/SensorHistorian.h>
#include <mps_voxels/image_utils.h>
#include <mps_voxels/ExperimentDir.h>
#include <mps_voxels/logging/DataLog.h>
#include <mps_voxels/logging/log_occupancy_data.h>
#include <mps_voxels/logging/log_sensor_history.h>
#include <mps_voxels/logging/log_cv_mat.h>
#include <mps_voxels/visualization/visualize_occupancy.h>
#include <mps_voxels/visualization/dispersed_colormap.h>

#include <mps_simulation/GazeboModel.h>
#include <mps_simulation/GazeboModelState.h>
#include <mps_simulation/paths.h>
#include <mps_simulation/loading.h>
#include <mps_test/CPUPixelizer.h>
#include <mps_test/ROSVoxelizer.h>
#include <tf_conversions/tf_eigen.h>

#include <rviz_cinematographer_msgs/CameraTrajectory.h>
#include <rviz_cinematographer_msgs/EnableDisplays.h>
#include <rviz_cinematographer_msgs/Record.h>
#include <rviz_cinematographer_msgs/Finished.h>

#include <tf/transform_broadcaster.h>
#include <std_msgs/Float64.h>

#include <opencv2/highgui.hpp>

#include <regex>
#include <boost/filesystem.hpp>

using VoxelColormap = std::map<mps::VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>;

using namespace mps;


void extend(VoxelColormap& cmap, const mps::VoxelRegion::VertexLabels& labels, std::mt19937& rng)
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

	Metrics(const OccupancyData& hypothesis, const OccupancyData& truth, const VoxelColormap& cmapGT, std::mt19937& rng)
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


cv::Mat colorByLabel(const cv::Mat& input, const VoxelColormap& colormap)
{
	mps::Colormap cv_cmap;
	for (const auto& c : colormap)
	{
		cv_cmap.emplace(c.first, cv::Point3_<uint8_t>(255.0f*c.second.b, 255.0f*c.second.g, 255.0f*c.second.r));
	}
	return colorByLabel(input, cv_cmap);
}

enum Technique
{
	GROUND_TRUTH,
	MERGED,
	SAMPLED,
	TRACKED
};

std::vector<std::string> TECHNIQUE_NAMES{
	"GROUND_TRUTH",
	"MERGED",
	"SAMPLED",
	"TRACKED"
};

class ViewController
{
public:
	mutable bool isFinished = true;

	ros::Publisher enablePub;
	ros::Publisher recordPub;
	ros::Publisher trajectoryPub;
	ros::Subscriber finishedSub;

	rviz_cinematographer_msgs::CameraTrajectory trajectory;

	static std::string filename(const std::string& experimentID, const Technique technique, const int generation, const int particle, const int stage)
	{
		const std::string particleStr = (particle == -1) ? "" : "_" + std::to_string(particle);
		const std::string stageStr = (stage == -1) ? "" : "_" + std::to_string(stage);
		return "/tmp/viz_" + experimentID + "_" + TECHNIQUE_NAMES[technique] + "_" + std::to_string(generation) + particleStr + stageStr + ".avi";
	}

	ViewController(const std::string& trajectoryFile)
	{
		ros::NodeHandle nh;

		YAML::Node traj = YAML::LoadFile(trajectoryFile);
		trajectory.interaction_disabled = true;
		trajectory.mouse_interaction_mode = 0;
		trajectory.allow_free_yaw_axis = false;
		trajectory.target_frame = "table_surface";
		for (YAML::iterator it = traj["trajectory"].begin(); it != traj["trajectory"].end(); ++it)
		{
			const YAML::Node& pose = *it;
			rviz_cinematographer_msgs::CameraMovement movement;
			movement.eye.header.frame_id = trajectory.target_frame;
//			movement.eye.header.stamp = stamp;
			movement.eye.point.x = pose["eye"]["x"].as<double>();
			movement.eye.point.y = pose["eye"]["y"].as<double>();
			movement.eye.point.z = pose["eye"]["z"].as<double>();
			movement.focus.header.frame_id = trajectory.target_frame;
//			movement.focus.header.stamp = stamp;
			movement.focus.point.x = pose["focus"]["x"].as<double>();
			movement.focus.point.y = pose["focus"]["y"].as<double>();
			movement.focus.point.z = pose["focus"]["z"].as<double>();
			movement.up.header.frame_id = trajectory.target_frame;
//			movement.up.header.stamp = stamp;
			movement.up.vector.x = 0.0;
			movement.up.vector.y = 0.0;
			movement.up.vector.z = 1.0;
			movement.interpolation_speed = rviz_cinematographer_msgs::CameraMovement::WAVE;
			movement.transition_duration = ros::Duration(1.0);
			trajectory.trajectory.push_back(movement);
		}

		enablePub = nh.advertise<rviz_cinematographer_msgs::EnableDisplays>("/rviz/enable_displays", 1, true);
		recordPub = nh.advertise<rviz_cinematographer_msgs::Record>("/rviz/record", 1, false);
		trajectoryPub = nh.advertise<rviz_cinematographer_msgs::CameraTrajectory>("/rviz/camera_trajectory", 1, false);
		finishedSub = nh.subscribe("/rviz/finished_rendering_trajectory", 1, &ViewController::finishedCallback, this);
	}

	void finishedCallback(const rviz_cinematographer_msgs::FinishedConstPtr& ptr)
	{
		isFinished = ptr->is_finished;
	}

	void renderVisuals(const std::string& experimentID, const Technique technique, const int generation, const int particle, const int stage)
	{
		ros::Time stamp = ros::Time::now();

		// Make sure we don't move on the first pass
		static bool first = true;
		if (first)
		{
			rviz_cinematographer_msgs::CameraTrajectory initTrajectory;
			trajectory.trajectory.push_back(trajectory.trajectory.front());
			first = false;
			trajectoryPub.publish(trajectory);
		}

		rviz_cinematographer_msgs::EnableDisplays displays;
//		for (int i = 0; i < 5; ++i)
//		{
//			if (i == particle)
//			{
//				displays.to_enable.push_back("VoxelData" + std::to_string(i));
//			}
//			else
//			{
//				displays.to_disable.push_back("VoxelData" + std::to_string(i));
//			}
//		}
//		if (particle != -1)
//		{
//			displays.to_disable.push_back("VoxelDataGT");
//		}
//		else
//		{
//			displays.to_enable.push_back("VoxelDataGT");
//		}
		enablePub.publish(displays);

		ros::Duration(0.5).sleep();

		rviz_cinematographer_msgs::Record record;
		record.do_record = true;
		record.add_watermark = false;
		record.compress = true;
		record.frames_per_second = 60;
		record.path_to_output = filename(experimentID, technique, generation, particle, stage);
		recordPub.publish(record);

		trajectoryPub.publish(trajectory);

		isFinished = false;
		for (int i = 0; i < 100; ++i)
		{
			if (isFinished) { break; }
			usleep(100000);
		}
	}
};

}

Eigen::Isometry3d getPose(const SensorHistoryBuffer& buffer, const std::string& globalFrame,
                          const std::string& objectFrame, ros::Time queryTime)
{
	std::string tfError;
	bool canTransform = buffer.tfs->canTransform(globalFrame, objectFrame, queryTime, &tfError);

	// Fuzzy lookup of time info
	if (!canTransform) // ros::Duration(5.0)
	{
		queryTime = queryTime + ros::Duration(0.5);
		canTransform = buffer.tfs->canTransform(globalFrame, objectFrame, queryTime, &tfError);
		if (!canTransform)
		{
			ROS_ERROR_STREAM("Failed to look up transform between '" << globalFrame << "' and '"
			                                                         << objectFrame << "' with error '"
			                                                         << tfError << "'.");
			throw std::runtime_error("Sadness.");
		}
	}

	tf::StampedTransform objectFrameInTableCoordinates;
	const auto temp = buffer.tfs->lookupTransform(objectFrame, globalFrame, queryTime);
	tf::transformStampedMsgToTF(temp, objectFrameInTableCoordinates);
	Eigen::Isometry3d pose;
	tf::transformTFToEigen(objectFrameInTableCoordinates.inverse(), pose);
	return pose;
}

#define VISUAL_ONLY true

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "occupancy_evaluation");
	ros::NodeHandle nh, pnh("~");

	std::string workingDir;
	bool gotDir = pnh.getParam("experiment_directory", workingDir);
	if (!gotDir)
	{
		ROS_FATAL_STREAM("No experiment directory provided to evaluate.");
		return -1;
	}
	if (!boost::filesystem::exists(workingDir))
	{
		ROS_FATAL_STREAM("Provided experiment directory '" << workingDir << "' does not exist.");
		return -1;
	}
	if (!boost::filesystem::is_directory(workingDir))
	{
		ROS_FATAL_STREAM("Provided path '" << workingDir << "' is not a directory.");
		return -1;
	}

	std::string trajectoryFile;
	gotDir = pnh.getParam("trajectory_file", trajectoryFile);
	if (!gotDir)
	{
		ROS_FATAL_STREAM("No trajectory provided.");
		return -1;
	}
	if (!boost::filesystem::exists(trajectoryFile))
	{
		ROS_FATAL_STREAM("Provided trajectory '" << trajectoryFile << "' does not exist.");
		return -1;
	}

	const std::string experimentID = boost::filesystem::path(workingDir).branch_path().leaf().string();

//	const std::string workingDir = "/home/kunhuang/mps_ws/src/mps_pipeline/mps_test_data/2020-06-24/";
//	const std::string workingDir = "/tmp/scene_explorer/2020-06-12T18:50:08.037350/"; // cubes?
//	const std::string workingDir = "/tmp/scene_explorer/2020-06-12T18:08:10.469214/"; // books
//	const std::string workingDir = "/tmp/scene_explorer/2020-06-10T23:35:24.412039/";
//	const std::string workingDir = "/tmp/scene_explorer/2020-06-08T21:57:37.545002/";
	const std::regex my_filter( "particle_([0-9]+)_([0-9]+)\\.bag" );

	const std::string primaryStageDir = workingDir + ExperimentDir::checkpoints[ExperimentDir::bestGuessIndex];
	int numGenerations = 0;
	int numParticles = 0;
	boost::filesystem::directory_iterator end_itr; // Default ctor yields past-the-end
	for (boost::filesystem::directory_iterator i(primaryStageDir); i != end_itr; ++i )
	{
		// Skip if not a file
		if( !boost::filesystem::is_regular_file( i->status() ) ) continue;

		std::smatch what;

		// Skip if no match:
		if(!std::regex_match(i->path().filename().string(), what, my_filter)) continue;

		numGenerations = std::max(numGenerations, std::stoi(what[1])+1);
		numParticles = std::max(numParticles, std::stoi(what[2])+1);
	}

	// Load experiment parameters
	if (!boost::filesystem::exists(workingDir + "rosparam.yaml"))
	{
		ROS_FATAL_STREAM("Working directory '" << workingDir << "' does not contain a parameter file.");
		return -1;
	}
	YAML::Node rosparams = YAML::LoadFile(workingDir + "rosparam.yaml");

	std::shared_ptr<mps::VoxelRegion> region = std::make_shared<mps::VoxelRegion>(mps::VoxelRegionBuilder::build(rosparams["scene_explorer"]["roi"]));
	const std::string globalFrame = region->frame_id;
	std_msgs::Header header;
	header.frame_id = globalFrame;

	std::random_device rd;
	int seed = rd(); //0;
	std::mt19937 rng = std::mt19937(seed);

	// Publishers for visualizations
	tf::TransformBroadcaster tb;
	ros::Publisher particlePubGT = nh.advertise<visualization_msgs::MarkerArray>("visualization"/*_gt*/, 1, true);
	std::vector<std::shared_ptr<ros::Publisher>> particlePubs;
	for (int p = 0; p < numParticles; ++p)
	{
		particlePubs.emplace_back(std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("visualization"/* + std::to_string(p)*/, 1, true)));
	}
	ros::Publisher camInfoPub = nh.advertise<sensor_msgs::CameraInfo>("/kinect2_victor_head/hd/camera_info", 1, true);
	ViewController vc(trajectoryFile);

	#if !VISUAL_ONLY
	const std::string evalFilename = workingDir + "eval.csv";
	{
		std::fstream out(evalFilename, std::ios::out);
		out << "Generation,Stage,Particle,A3,S3,A2,S2" << std::endl;
	}

	// Load Gazebo world file and get meshes
	const std::string gazeboWorldFilename = parsePackageURL(rosparams["simulation_world"].as<std::string>());

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

	// Pass meshes to renderer
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

	// Create state and observation generators
	std::unique_ptr<ScenePixelizer> pixelizer = std::make_unique<CPUPixelizer>(tmpModels);
	std::unique_ptr<SceneVoxelizer> voxelizer = std::make_unique<ROSVoxelizer>(tmpModels);

	// Create colormap for ground truth
	VoxelColormap cmapGT;
	auto colors = dispersedColormap(shapeModels.size());
	for (VoxelColormap::key_type i = 0; i < static_cast<VoxelColormap::key_type>(shapeModels.size()); ++i)
	{
		cmapGT.emplace(i+1, colors[i]);
	}
	cmapGT.emplace(VoxelRegion::FREE_SPACE, std_msgs::ColorRGBA());
	#endif
//	cv::namedWindow("Ground Truth", CV_WINDOW_NORMAL);
//	cv::namedWindow("Mask", CV_WINDOW_NORMAL);

	for (int generation = 0; generation < numGenerations; ++generation)
	{
		const bool isFinalGeneration = (generation >= numGenerations-1);
		//-------------------------------------------------------------------------
		// Ground Truth - State
		//-------------------------------------------------------------------------
		const std::string bufferFilename =
			workingDir + "buffer_"
			+ std::to_string(isFinalGeneration ? generation-1 : generation) + ".bag";
		mps::DataLog sensorLog(bufferFilename, {"buffer"}, rosbag::BagMode::Read);
		mps::SensorHistoryBuffer motionData = sensorLog.load<mps::SensorHistoryBuffer>("buffer");

		// Compute poses from buffer
		const ros::Time queryTime = (isFinalGeneration) ? ros::Time(0)
		                                                : motionData.rgb.begin()->second->header.stamp + ros::Duration(1.0);
		Eigen::Isometry3d worldTcamera = getPose(motionData, globalFrame, motionData.cameraModel.tfFrame(), queryTime);

		#if !VISUAL_ONLY

		std::vector<Eigen::Isometry3d> statePoses;
		for (const auto& pair : shapeModels)
		{
			const std::string objectFrame = "mocap_" + pair.first;
			statePoses.push_back(getPose(motionData, globalFrame, objectFrame, queryTime));
		}

		mps::VoxelRegion::VertexLabels labels = voxelizer->voxelize(*region, statePoses);
		mps::OccupancyData b(region, labels);

		const cv::Rect objectsROIGT = occupancyToROI(b, motionData.cameraModel, worldTcamera);
		cv::Mat segGT = rayCastOccupancy(b, motionData.cameraModel, worldTcamera, objectsROIGT);

		// Send the camera info
		for (int i = 0; i < 5; ++i)
		{
			ros::Time stamp = ros::Time::now();

			tf::StampedTransform cTw;
			tf::transformEigenToTF(worldTcamera.inverse(Eigen::Isometry), cTw);
			cTw.stamp_ = stamp;
			cTw.frame_id_ = motionData.cameraModel.tfFrame();
			cTw.child_frame_id_ = globalFrame;
			tb.sendTransform(cTw);

			sensor_msgs::CameraInfo camInfo = motionData.cameraModel.cameraInfo();
			camInfo.header.stamp = stamp;
			camInfoPub.publish(camInfo);
			usleep(10000);
		}

		header.stamp = ros::Time::now();
		particlePubGT.publish(mps::visualize(b, header, cmapGT));

		vc.renderVisuals(experimentID, Technique::GROUND_TRUTH, generation, -1, -1);

//		cv::imshow("Ground Truth", colorByLabel(segGT, cmapGT));

		#endif

//		const std::vector<size_t> stage_order{0,3,4,2};
		const std::vector<size_t> stage_order{0,2,5};
		const std::map<size_t, Technique> tmap{{0, Technique::SAMPLED}, {2, Technique::MERGED}, {5, Technique::TRACKED}};
		int bestParticle = -1;
		double bestParticleVal = -std::numeric_limits<double>::infinity();
		for (const size_t stage : stage_order)
		{
			for (int p = 0; p < numParticles; ++p)
			{
				const std::string particleFilename =
					workingDir + ExperimentDir::checkpoints[stage] + "/particle_"
					+ std::to_string(generation) + "_"
					+ std::to_string(p) + ".bag";
				if (!boost::filesystem::exists(particleFilename))
				{
					ROS_WARN_STREAM("Could not find '" << particleFilename << "'");
					continue;
				}

				mps::DataLog particleLog(particleFilename, {"particle"}, rosbag::BagMode::Read);
				mps::OccupancyData a = particleLog.load<mps::OccupancyData>("particle");

				if (tmap.at(stage) == MERGED)
				{
					auto f = particleLog.load<std_msgs::Float64>("weight");
					if (f.data > bestParticleVal)
					{
						bestParticle = p;
						bestParticleVal = f.data;
					}
				}

				if (a.vertexState.size() != a.voxelRegion->num_vertices())
				{
					throw std::logic_error("Fake news (a)!");
				}

				// Image region of objects currently
				const cv::Rect objectsROI = occupancyToROI(a, motionData.cameraModel, worldTcamera);
				cv::Mat segParticle = rayCastOccupancy(a, motionData.cameraModel, worldTcamera, objectsROI);

				#if !VISUAL_ONLY
				mps::Metrics metrics(a, b, cmapGT, rng);

				JaccardMatch J2(segParticle, segGT);

				std::cerr << generation << ": " << p << std::endl;
				std::cerr << "\t" << metrics.match.match.first << "\t" << metrics.match.symmetricCover()
				          << std::endl;

				{
					std::fstream out(evalFilename, std::ios::app);
					out << generation << ","
						<< stage << ","
						<< p << ","
						<< metrics.match.match.first << ","
						<< metrics.match.symmetricCover() << ","
						<< J2.match.first << ","
						<< J2.symmetricCover() << std::endl;
				}

				header.stamp = ros::Time::now();
//				particlePubGT.publish(mps::visualize(b, header, cmapGT));
				particlePubs[p]->publish(mps::visualize(a, header, metrics.cmapA));
				#else
				VoxelColormap cmap;
				cmap.emplace(VoxelRegion::FREE_SPACE, std_msgs::ColorRGBA());
				for (const auto label : a.uniqueObjectLabels)
				{
					cmap.emplace(label.id, randomColorMsg(rng));
				}
				particlePubs[p]->publish(mps::visualize(a, header, cmap));

				#endif

				// Send the camera info
				for (int i = 0; i < 5; ++i)
				{
					ros::Time stamp = ros::Time::now();

					tf::StampedTransform cTw;
					tf::transformEigenToTF(worldTcamera.inverse(Eigen::Isometry), cTw);
					cTw.stamp_ = stamp;
					cTw.frame_id_ = motionData.cameraModel.tfFrame();
					cTw.child_frame_id_ = globalFrame;
					tb.sendTransform(cTw);

					sensor_msgs::CameraInfo camInfo = motionData.cameraModel.cameraInfo();
					camInfo.header.stamp = stamp;
					camInfoPub.publish(camInfo);
					usleep(10000);
				}

				vc.renderVisuals(experimentID, tmap.at(stage), generation, p, stage);

			}
		}

		if (generation > 0)
		{
			std::fstream out("/tmp/" + experimentID + ".sh", std::ios::app);
			out << "ffmpeg \\\n";
			#if !VISUAL_ONLY
			out << "\t-i " + ViewController::filename(experimentID, GROUND_TRUTH, generation, -1, -1) + "\\\n";
			out << "\t-i " + ViewController::filename(experimentID, MERGED, generation, bestParticle, 2) + "\\\n";
			out << "\t-i " + ViewController::filename(experimentID, SAMPLED, generation, 0, 0) + "\\\n";
			out << "\t-i " + ViewController::filename(experimentID, TRACKED, generation, 0, 5) + "\\\n";
			out << "\t-filter_complex \"\n"
			       "\t\tnullsrc=size=1920x1080 [base];\n"
			       "\t\t[0:v] setpts=PTS-STARTPTS, scale=960x540 [upperleft];\n"
			       "\t\t[1:v] setpts=PTS-STARTPTS, scale=960x540 [upperright];\n"
			       "\t\t[2:v] setpts=PTS-STARTPTS, scale=960x540 [lowerleft];\n"
			       "\t\t[3:v] setpts=PTS-STARTPTS, scale=960x540 [lowerright];\n"
			       "\t\t[base][upperleft] overlay=shortest=1 [tmp1];\n"
			       "\t\t[tmp1][upperright] overlay=shortest=1:x=960 [tmp2];\n"
			       "\t\t[tmp2][lowerleft] overlay=shortest=1:y=540 [tmp3];\n"
			       "\t\t[tmp3][lowerright] overlay=shortest=1:x=960:y=540\n"
			       "\t\" \\" << std::endl;
			#else
			out << "\t-i " + ViewController::filename(experimentID, MERGED, generation, bestParticle, 2) + "\\\n";
			out << "\t-i " + ViewController::filename(experimentID, SAMPLED, generation, 0, 0) + "\\\n";
			out << "\t-i " + ViewController::filename(experimentID, TRACKED, generation, 0, 5) + "\\\n";
			out << "\t-filter_complex \"\n"
			       "\t\tcolor=size=1920x1080:c=black [base];\n"
			       "\t\t[0:v] setpts=PTS-STARTPTS, scale=960x540 [upperright];\n"
			       "\t\t[1:v] setpts=PTS-STARTPTS, scale=960x540 [lowerleft];\n"
			       "\t\t[2:v] setpts=PTS-STARTPTS, scale=960x540 [lowerright];\n"
			       "\t\t[base][upperright] overlay=shortest=1:x=960 [tmp1];\n"
			       "\t\t[tmp1][lowerleft] overlay=shortest=1:y=540 [tmp2];\n"
			       "\t\t[tmp2][lowerright] overlay=shortest=1:x=960:y=540\n"
			       "\t\" \\" << std::endl;
			#endif
			out << "\t-c:v libx264 " << experimentID << "_" << generation << ".mkv;" << std::endl;

		}

	}

	return 0;
}
