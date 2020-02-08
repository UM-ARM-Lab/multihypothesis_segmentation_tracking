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

#include "mps_voxels/SensorHistorian.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/Ultrametric.h"
#include "mps_voxels/SceneCut.h"
#include "mps_voxels/MCMCTreeCut.h"
#include "mps_voxels/JaccardMatch.h"

#include "mps_voxels/util/package_paths.h"
#include "mps_voxels/logging/DataLog.h"
#include "mps_voxels/logging/log_segmentation_info.h"
#include "mps_voxels/logging/log_sensor_history.h"
#include "mps_voxels/logging/log_cv_mat.h"

#include "mps_voxels/image_utils.h"
#include <opencv2/highgui.hpp>

#include <fstream>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

using namespace mps;


cv::Mat relabelCut(const Ultrametric& um, const ValueTree& T, const cv::Mat& labels, const TreeCut& cut)
{
	std::map<int, int> index_to_label;
	for (const auto& pair : um.label_to_index)
	{
		index_to_label.insert({pair.second, pair.first});
	}

	int label = 0;

//	cv::Mat segmentation = cv::Mat::zeros(labels.size(), labels.type());
	cv::Mat segmentation = cv::Mat(labels.size(), labels.type());
	for (const auto node : cut)
	{
		++label;
		std::set<int> children;
		descendants(T, node, children);
		children.insert(node); // This cut node could be a leaf as well
		for (const auto c : children)
		{
			if (T.children[c].empty()) // Leaf node
			{
				auto mask = (labels == index_to_label.at(c));
				segmentation.setTo(label, mask);
			}
		}
	}

	return segmentation;
}

namespace mps
{
using Colormap = std::map<uint16_t, cv::Point3_<uint8_t>>;
}

Colormap createColormap(const std::set<uint16_t>& labels, std::default_random_engine& re)
{
	Colormap colormap;
	std::uniform_int_distribution<> dis(0, 256);
	for (auto label : labels)
	{
		colormap[label] = cv::Point3_<uint8_t>(dis(re), dis(re), dis(re));
	}
	return colormap;
}
Colormap createColormap(const cv::Mat& labels, std::default_random_engine& re)
{
	return createColormap(unique(labels), re);
}

void extendColormap(Colormap& colormap, const std::set<uint16_t>& labels, std::default_random_engine& re)
{
	std::uniform_int_distribution<> dis(0, 256);
	for (auto label : labels)
	{
		auto iter = colormap.find(label);
		if (iter == colormap.end())
		{
			colormap[label] = cv::Point3_<uint8_t>(dis(re), dis(re), dis(re));
		}
	}
}
void extendColormap(Colormap& colormap, const cv::Mat& labels, std::default_random_engine& re)
{
	extendColormap(colormap, unique(labels), re);
}

//template <typename T>
//using DataGeneratorFn = std::function<bool(T&)>;

template <typename T>
using DataGeneratorFn = bool(T&);

template <typename T>
bool loadOrGenerateData(const std::string& bagFileName, const std::string& channelName,
                        T& data, DataGeneratorFn<T> generator, bool cacheDataIfGenerated = true)
{
	// Try to load the data from file
	if (fs::exists(bagFileName))
	{
		std::shared_ptr<DataLog> log = std::make_shared<DataLog>(bagFileName, std::unordered_set<std::string>{channelName}, rosbag::BagMode::Read);
		try
		{
			if (log->load(channelName, data))
			{
				return true;
			}
		}
		catch (...)
		{
			// proceed to generator
		}
	}

	// Call the generator function
	bool res = generator(data);
	if (!res) { return false; }

	// Write back out to bag file
	if (cacheDataIfGenerated)
	{
		auto mode = fs::exists(bagFileName) ? rosbag::BagMode::Append : rosbag::BagMode::Write;
		std::shared_ptr<DataLog> log = std::make_shared<DataLog>(bagFileName, std::unordered_set<std::string>{channelName}, mode);
		log->log(channelName, data);
	}
	return true;
}


const std::string testDirName = "package://mps_test_data/";

bool generateSensorHistory(SensorHistoryBuffer& buff)
{
	SensorHistorian::SubscriptionOptions opts;
	opts.hints = image_transport::TransportHints();
	auto historian = std::make_unique<SensorHistorian>(500, opts);
	historian->startCapture();
	for (int attempt = 0; attempt < 1000; ++attempt)
	{
		ros::Duration(0.1).sleep();
		if (!historian->buffer.rgb.empty()) { break; }
	}
	historian->stopCapture();
	if (historian->buffer.rgb.empty())
	{
		ROS_ERROR_STREAM("Failed to get any data.");
		return false;
	}

	buff = historian->buffer;

	return true;
}

bool generateSegmentationInfo(SegmentationInfo& info)
{
//	std::string bagFileName = parsePackageURL(testDirName) + ros::this_node::getName() + ".bag";
	std::string bagFileName = parsePackageURL(testDirName) + "experiment_world.bag";
	SensorHistoryBuffer buff;
	bool res = loadOrGenerateData(bagFileName, "sensor_history", buff, generateSensorHistory, true);
	if (!res)
	{
		ROS_ERROR_STREAM("Failed to get sensor history.");
		return false;
	}

	ros::NodeHandle nh;
	auto segmentationClient = std::make_shared<RGBDSegmenter>(nh);
	auto si = segmentationClient->segment(*buff.rgb.begin()->second, *buff.depth.begin()->second, buff.cameraModel.cameraInfo());
	if (!si)
	{
		ROS_ERROR_STREAM("Failed to get segmentation.");
		return false;
	}

	info = *si;
	return true;
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "segmentation_sampling");
	ros::NodeHandle nh, pnh("~");

	// Check for existence of mps_test_data
	if (!(fs::is_directory(parsePackageURL(testDirName)) && fs::exists(parsePackageURL(testDirName))))
	{
		ROS_ERROR_STREAM("Unable to find test data directory.");
		return -1;
	}

//	std::string bagFileName = parsePackageURL(testDirName) + ros::this_node::getName() + ".bag";
	std::string bagFileName = parsePackageURL(testDirName) + "experiment_world.bag";

	auto si = std::make_shared<SegmentationInfo>();

	bool res = loadOrGenerateData(bagFileName, "segmentation_info", *si, generateSegmentationInfo, true);
	if (!res)
	{
		ROS_ERROR_STREAM("Failed to get segmentation.");
		return -1;
	}

	// Load ground truth labels
	std::shared_ptr<DataLog> log = std::make_shared<DataLog>(bagFileName, std::unordered_set<std::string>{"ground_truth/labels"}, rosbag::BagMode::Read);
	cv::Mat gt_labels;
	try
	{
		sensor_msgs::Image im;
		if (!log->load("ground_truth/labels", im))
		{
			ROS_FATAL_STREAM("Failed to load ground truth.");
			return -1;
		}
		gt_labels = fromMessage(im);
		assert(gt_labels.type() == CV_16UC1);
	}
	catch (...)
	{
		ROS_FATAL_STREAM("Exception while loading ground truth.");
		return -1;
	}


//	JaccardMatch jaccardMatch(si->labels, si->labels);
//	auto pairing = jaccardMatch(si->objectness_segmentation->image, si->objectness_segmentation->image);
//	const auto& pairing = jaccardMatch.match;

	std::random_device rd;
	std::default_random_engine re(0);// (rd());
	Colormap colormap = createColormap(gt_labels, re);


	Ultrametric um(si->ucm2, si->labels2);
	SceneCut sc;
	ValueTree T = sc.process(um, si->labels);

	// TODO: Cut the ValueTree into a value forest based on free space

	auto cutStar = optimalCut(T);
	double vStar = value(T, cutStar.second);
	const double sigmaSquared = vStar*vStar;
	const int seed = 1;

	MCMCTreeCut mcmc(T, sigmaSquared);

//	std::random_device rd;
//	std::default_random_engine re(rd());

	cutStar = mcmc.sample(re, Belief<TreeCut>::SAMPLE_TYPE::MAXIMUM);

	// Validate this cut
	for (size_t n = 0; n < T.parent.size(); ++n)
	{
		if (T.children[n].empty())
		{
			auto an = ancestors(T, n);
			an.insert(n);
			std::vector<int> crossover;
			std::set_intersection(an.begin(), an.end(), cutStar.second.begin(), cutStar.second.end(), std::back_inserter(crossover));
			assert(crossover.size() == 1); // A leaf node should have exactly one leaf node that is a parent.
		}
	}

	cv::namedWindow("Image", cv::WINDOW_NORMAL);
	cv::namedWindow("Labels", cv::WINDOW_NORMAL);
	cv::namedWindow("Contours", cv::WINDOW_NORMAL);
	cv::namedWindow("Objectness", cv::WINDOW_NORMAL);
	cv::namedWindow("Segmentation", cv::WINDOW_NORMAL);
	cv::namedWindow("Ground Truth", cv::WINDOW_NORMAL);

	cv::imshow("Image", si->rgb);
	cv::imshow("Ground Truth", colorByLabel(gt_labels, colormap));
	cv::imshow("Labels", colorByLabel(si->labels));
	cv::imshow("Contours", si->display_contours);
	cv::theRNG().state = seed;
	cv::imshow("Objectness", colorByLabel(si->objectness_segmentation->image));

	std::string dir = "/tmp/segmentation/";
	cv::imwrite(dir + "image.png", si->rgb);
	cv::imwrite(dir + "contours.png", si->display_contours);
	cv::theRNG().state = seed;
	cv::imwrite(dir + "objectness.png", colorByLabel(si->objectness_segmentation->image));
	cv::imwrite(dir + "ground.png", colorByLabel(gt_labels, colormap));

	std::cerr << "# original objects: " << unique(si->objectness_segmentation->image).size() << std::endl;
	std::cerr << "# new objects: " << cutStar.second.size() << std::endl;

	cv::theRNG().state = seed;
	auto seg = relabelCut(um, T, si->labels, cutStar.second);
	auto disp = colorByLabel(seg);

	cv::imshow("Segmentation", disp);
	cv::imwrite(dir + "cStar.png", disp);

	cv::Mat maskInv = (gt_labels == 0); // All irrelevant pixels
	cv::imshow("Mask", maskInv);

	cv::waitKey(0);

//	mcmc.sigmaSquared = 1e10*vStar*vStar; // With proposal ratio
//	mcmc.nTrials = 100;
	mcmc.sigmaSquared = 0.2*vStar*vStar; // The larger this number, the more permissive exploration is allowed
	mcmc.nTrials = 25;

	std::ofstream csv;
	csv.open(dir + "data.csv", std::ios::out);

	csv << "Trial\tlogp(cut)\tJaccard\tCover" << std::endl;

	for (int i = 0; i < 100; ++i)
	{
		std::pair<double, TreeCut> cut;
		if (0 == i)
		{
			cut = cutStar;
		}
		else
		{
			cut = mcmc.sample(re, Belief<TreeCut>::SAMPLE_TYPE::RANDOM);
		}
		seg = relabelCut(um, T, si->labels, cut.second);
		seg.setTo(0, maskInv);
		JaccardMatch jaccard(gt_labels, seg);
		Colormap cmap = colormap;
		extendColormap(cmap, seg, re);
		disp = colorByLabel(relabel(seg, jaccard.match.second.right), cmap);
		cv::imshow("Segmentation", disp);
		cv::imwrite(dir + "c" + std::to_string(i) + ".png", disp);
		csv << i << "\t" << cut.first << "\t" << jaccard.match.first << "\t" << jaccard.symmetricCover() << std::endl;

//		cv::waitKey(10);
	}

	csv.close();

	cv::waitKey(0);
	return 0;

}