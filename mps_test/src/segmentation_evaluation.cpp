//
// Created by kunhuang on 6/29/20.
//

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
#include "mps_voxels/logging/log_segmentation_info.h"
#include <mps_voxels/visualization/dispersed_colormap.h>
#include "mps_voxels/image_output.h"

#include <mps_simulation/GazeboModel.h>
#include <mps_simulation/GazeboModelState.h>
#include <mps_test/CPUPixelizer.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/highgui.hpp>

#include <regex>
#include <boost/filesystem.hpp>

using VoxelColormap = std::map<mps::VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>;

using namespace mps;

const std::string workingDir = "/home/kunhuang/mps_ws/src/mps_pipeline/mps_test_data/2020-06-29/";

namespace cv
{
bool operator<(const cv::Vec3b &lhs, const cv::Vec3b &rhs)
{
	for (int i = 0; i < 3; ++i)
	{
		if (lhs(i) < rhs(i)) return true;
		if (lhs(i) > rhs(i)) return false;
	}
	return false;
}
}

namespace mps
{
cv::Mat colorSeg2LabelIm(const image_geometry::PinholeCameraModel& cameraModel, cv::Mat& segGT, const cv::Rect& roi)
{
	std::map<cv::Vec3b, int> counter, colorMapInv;
	cv::Mat labels = cv::Mat::zeros(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_16U);
//	cv::Mat segGTHSV;
//	cvtColor(segGT, segGTHSV, CV_BGR2HSV);
//	std::map<uchar, uint16_t> H2label;
	uint16_t label = 1000;
	int numObjs = 0;
//#pragma omp parallel for
	for (int v = roi.y; v < roi.y+roi.height; ++v)
	{
		for (int u = roi.x; u < roi.x + roi.width; ++u)
		{
			const auto &val = segGT.at<cv::Vec3b>(v - roi.y, u - roi.x);
			counter[val]++;
		}
	}



	for (int v = roi.y; v < roi.y+roi.height; ++v)
	{
		for (int u = roi.x; u < roi.x + roi.width; ++u)
		{
			const auto& val = segGT.at<cv::Vec3b>(v-roi.y, u-roi.x);
			if (counter.at(val) < 20*20) // 40*40 for real_simple
			{
//				std::cerr << "Rejected (" << (int)val(0) << ", " << (int)val(1) << "," << (int)val(2) << ")." << std::endl;
				continue;
			}

			if (val(0) + val(1) + val(2) < 5*3
				|| val(0) + val(1) + val(2) > 250*3)
			{
				labels.at<uint16_t>(v, u) = 0;
				continue;
			}

			auto iter = colorMapInv.find(val);
			if (iter == colorMapInv.end())
			{
				iter = colorMapInv.emplace(val, ++numObjs).first;
			}

			labels.at<uint16_t>(v, u) = iter->second;
//			cv::Vec3b intensity2 = segGTHSV.at<cv::Vec3b>(v-roi.y, u-roi.x);
//			uchar H = intensity2.val[0];
//			uchar S = intensity2.val[1];
//			uchar V = intensity2.val[2];
////			if (S==0 || V == 255) continue;
//			if (S >= 200)
//			{
//				if (H2label.find(H) != H2label.end())
//				{
//					labels.at<uint16_t>(v, u) = H2label.at(H);
//				}
//				else
//				{
//					H2label[H] = label;
//					labels.at<uint16_t>(v, u) = label;
//					label+=100;
//					numObjs++;
//				}
//			}
		}
	}
	std::cerr << "num of segments in the image: " << numObjs << std::endl;
	return labels;
}


cv::Mat colorByLabel(const cv::Mat& input, const VoxelColormap& colormap)
{
	mps::Colormap cv_cmap;
	for (const auto& c : colormap)
	{
		cv_cmap.emplace(c.first, cv::Point3_<uint8_t>(255.0f*c.second.b, 255.0f*c.second.g, 255.0f*c.second.r));
	}
	return colorByLabel(input, cv_cmap);
}

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

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "occupancy_evaluation");
	ros::NodeHandle nh, pnh("~");

//	std::string dir;
//	bool gotDir = pnh.getParam("experiment_directory", dir);
//	if (!gotDir)
//	{
//		ROS_FATAL_STREAM("No experiment directory provided to evaluate.");
//		return -1;
//	}

	const std::regex my_filter( "particle_(.+)_(.+)\\.bag" );

	const std::string evalFilename = workingDir + "eval.csv";
	{
		std::fstream out(evalFilename, std::ios::out);
		out << "Generation,Stage,Particle,A2,S2,C2" << std::endl;
	}

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

	std::random_device rd;
	int seed = rd(); //0;
	std::mt19937 rng = std::mt19937(seed);

	ros::Publisher particlePubGT = nh.advertise<visualization_msgs::MarkerArray>("visualization_gt", 1, true);
	std::vector<std::shared_ptr<ros::Publisher>> particlePubs;
	for (int p = 0; p < numParticles; ++p)
	{
		particlePubs.emplace_back(std::make_shared<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("visualization_" + std::to_string(p), 1, true)));
	}


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

		// Publish images here
		// Compute poses from buffer
		const ros::Time queryTime = (isFinalGeneration) ? ros::Time(0)
		                                                : motionData.rgb.begin()->second->header.stamp + ros::Duration(1.0);
		Eigen::Isometry3d worldTcamera = getPose(motionData, globalFrame, motionData.cameraModel.tfFrame(), queryTime);


		//// load ROI of rgb image from segInfo
		const std::string segInfoFilename =
			workingDir + "segInfo_"
			+ std::to_string(generation) + ".bag";
		SegmentationInfo segData;
		{
			DataLog loader(segInfoFilename, {}, rosbag::bagmode::Read);
			loader.activeChannels.insert("segInfo");
			segData = loader.load<SegmentationInfo>("segInfo");
			std::cerr << "Successfully loaded segInfo." << std::endl;
		}
		const cv::Rect objectsROIGT = segData.roi;

		//// load segGT from "seg0.png"
//		cv::Mat segGT = rayCastOccupancy(b, motionData.cameraModel, worldTcamera, objectsROIGT);
		const std::string segGTFilename =
			workingDir + "seg"
			+ std::to_string(generation) + /*"_filtered.png"*/ ".png";
		cv::Mat segGT = cv::imread(segGTFilename, cv::IMREAD_COLOR);
		cv::Mat labels = colorSeg2LabelIm(motionData.cameraModel, segGT, segData.roi);
//		cv::imwrite(workingDir+"labels"+ std::to_string(generation) + ".png", colorByLabel(labels));

		IMSHOW("segmentation", colorByLabel(labels));
		sleep(1);
/*
		cv::Mat segGTHSV;
		cvtColor(segGT, segGTHSV, CV_BGR2HSV);
		for (int y = 200; y < 250; ++y)
		{
			for (int x = 400; x < 450; ++x)
			{
				cv::Vec3b intensity = segGT.at<cv::Vec3b>(y, x);
				uchar blue = intensity.val[0];
				uchar green = intensity.val[1];
				uchar red = intensity.val[2];
				std::cerr << blue << " " << green << " " << red << std::endl;

				cv::Vec3b intensity2 = segGTHSV.at<cv::Vec3b>(y, x);
				uchar H = intensity.val[0];
				uchar S = intensity.val[1];
				uchar V = intensity.val[2];
				std::cerr << H << " " << S << " " << V << std::endl;
			}
		}
*/
		//// put segGT into full frame label image
/*
		cv::Mat startMask = cv::Mat::zeros(buffer_out.rgb.begin()->second->image.size(), CV_8UC1);
		cv::Mat subwindow(startMask, seg_out.roi);
		subwindow = pair.first == seg_out.objectness_segmentation->image;
		masks.insert(masks.begin(), {steps.front(), startMask});
*/


//		cv::imshow("Ground Truth", colorByLabel(segGT, cmapGT));

//		const std::vector<size_t> stage_order{0,3,4,2};
		const std::vector<size_t> stage_order{0, ExperimentDir::bestGuessIndex};
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
					ROS_ERROR_STREAM("Could not find '" << particleFilename << "'");
					continue;
				}

				mps::DataLog particleLog(particleFilename, {"particle"}, rosbag::BagMode::Read);
				mps::OccupancyData a = particleLog.load<mps::OccupancyData>("particle");

				if (a.vertexState.size() != a.voxelRegion->num_vertices())
				{
					throw std::logic_error("Fake news (a)!");
				}

				// Image region of objects currently
				const cv::Rect objectsROI = occupancyToROI(a, motionData.cameraModel, worldTcamera);
				cv::Mat segParticle = rayCastOccupancy(a, motionData.cameraModel, worldTcamera, objectsROI);

//				cv::imwrite(workingDir+"particle"+ std::to_string(generation) + "_" + std::to_string(stage) + "_" + std::to_string(p) + ".png", colorByLabel(segParticle));

				cv::Mat segParticleROI = segParticle(segData.roi);
				cv::Mat labelsinROI = labels(segData.roi);
				cv::imwrite(workingDir+"labels"+ std::to_string(generation) + ".png", colorByLabel(labelsinROI));
				cv::imwrite(workingDir+"particle"+ std::to_string(generation) + "_" + std::to_string(stage) + "_" + std::to_string(p) + ".png", colorByLabel(segParticleROI));

//				JaccardMatch J2(segParticle, labels);
				JaccardMatch J2(segParticleROI, labelsinROI);

				std::cerr << generation << ": " << p << std::endl;

				{
					std::fstream out(evalFilename, std::ios::app);
					out << generation << ","
					    << stage << ","
					    << p << ","
					    << J2.match.first << ","
					    << J2.symmetricCover() << ","
					    << J2.cover() << std::endl;
				}


//				header.stamp = ros::Time::now();
//				particlePubs[p]->publish(mps::visualize(a, header, metrics.cmapA));
			}
		}
	}

	return 0;
}
