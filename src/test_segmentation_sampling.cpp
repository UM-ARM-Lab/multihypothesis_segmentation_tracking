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

#include "mps_voxels/ValueTree.hpp"
template <typename ValueTree>
cv::Mat relabelCut(const Ultrametric& um, const ValueTree& T, const cv::Mat& labels, const tree::TreeCut& cut)
{
	std::map<int, int> index_to_label;
	for (const auto& pair : um.label_to_index)
	{
		index_to_label.insert({pair.second, pair.first});
	}

	int label = 0;

	cv::Mat segmentation = cv::Mat::zeros(labels.size(), labels.type());
//	cv::Mat segmentation = cv::Mat(labels.size(), labels.type());
	for (const auto node : cut)
	{
		++label;
		std::set<int> children;
		tree::descendants(T, node, children);
		children.insert(node); // This cut node could be a leaf as well
		for (const auto c : children)
		{
			if (tree::children(T, c).empty()) // Leaf node
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
namespace tree
{

template <typename ValueTree>
SparseValueTree extractSubtree(const ValueTree& T, const std::set<NodeID>& leaves)
{
#ifndef NDEBUG
	// Verify leaves-only input
	for (const NodeID& n : leaves)
	{
		assert(T.children_.at(n).empty());
	}
#endif

	std::deque<NodeID> q; q.insert(q.end(), leaves.begin(), leaves.end());

	SparseValueTree subtree;
	for (; !q.empty(); q.pop_front())
	{
		const NodeID& n = q.front();
		const NodeID& p = parent(T, n);

		// Has n already been added?
		if (subtree.value_.find(n) != subtree.value_.end())
		{
			continue;
		}

		// Add the properties of this node
		subtree.value_.emplace(n, value(T, n));
		subtree.parent_.emplace(n, p);
		subtree.children_[n];
		if (n != p) // Don't add the root as a child of itself
		{
			subtree.children_[p].push_back(n);
		}

		// Has parent already been added?
		if (subtree.value_.find(p) == subtree.value_.end())
		{
			q.push_back(p);
		}
	}

#ifndef NDEBUG
	// Validation
	for (const auto& pair : subtree.value_)
	{
		const NodeID& n = pair.first;
		assert(subtree.parent_.find(n) != subtree.parent_.end()); // has parent
		assert(subtree.value_.find(subtree.parent_.at(n)) != subtree.value_.end()); // parent is valid
		assert(subtree.children_.find(n) != subtree.children_.end()); // has children array
		for (const NodeID& c : subtree.children_.at(n))
		{
			assert(subtree.value_.find(c) != subtree.value_.end());
		}
	}
#endif

	return subtree;
}

void compressTree(SparseValueTree& T)
{
	for (auto it = T.children_.cbegin(); it != T.children_.cend(); /* no increment */)
	{
		if (it->second.size() == 1)
		{
			const NodeID& n = it->first;
			const NodeID& c = it->second[0];
			const NodeID& p = parent(T, n);

			value(T, c) = std::max(value(T, n), value(T, c));
			if (n == p)
			{
				// We are the root
				parent(T, c) = c;
			}
			else
			{
				// Promote the only child
				parent(T, c) = p;
				auto& C = children(T, p);
				std::replace(C.begin(), C.end(), n, c);
			}

			// Suicide
			T.parent_.erase(n);
			T.value_.erase(n);
			it = T.children_.erase(it);
		}
		else
		{
			++it;
		}
	}
}

}
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

void reallySimpleImageMatch()
{
	std::vector<uint16_t> data1 = {1, 1, 1, 2, 2, 2,
	                               3, 3, 3, 4, 4, 4};
	cv::Mat img1(2, 6, CV_16UC1, data1.data());

	std::vector<uint16_t> data2 = {1, 1, 1, 1, 1, 1,
	                               2, 2, 2, 2, 2, 2};
	cv::Mat img2(2, 6, CV_16UC1, data2.data());

	std::vector<uint16_t> data3 = {1, 1, 1, 1, 1, 1,
	                               1, 1, 2, 2, 2, 2};
	cv::Mat img3(2, 6, CV_16UC1, data3.data());

	std::vector<uint16_t> data4 = {1, 1, 1, 1, 2, 2,
	                               3, 3, 4, 4, 4, 4};
	cv::Mat img4(2, 6, CV_16UC1, data4.data());

	cv::namedWindow("1", cv::WINDOW_NORMAL);
	cv::imshow("1", colorByLabel(img1));
	cv::namedWindow("2", cv::WINDOW_NORMAL);
	cv::imshow("2", colorByLabel(img2));
	cv::namedWindow("3", cv::WINDOW_NORMAL);
	cv::imshow("3", colorByLabel(img3));
	cv::namedWindow("4", cv::WINDOW_NORMAL);
	cv::imshow("4", colorByLabel(img4));

	JaccardMatch jaccard4(img1, img4);
	std::cerr << "1:4" << "\t" << jaccard4.match.first << "\t" << jaccard4.symmetricCover() << std::endl;

	JaccardMatch jaccard3(img1, img3);
	std::cerr << "1:3" << "\t" << jaccard3.match.first << "\t" << jaccard3.symmetricCover() << std::endl;

	JaccardMatch jaccard2(img1, img2);
	std::cerr << "1:2" << "\t" << jaccard2.match.first << "\t" << jaccard2.symmetricCover() << std::endl;

	JaccardMatch jaccard1(img1, img1);
	std::cerr << "1:1" << "\t" << jaccard1.match.first << "\t" << jaccard1.symmetricCover() << std::endl;

	cv::waitKey(0);
}


int main(int argc, char* argv[])
{
//	reallySimpleImageMatch();
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
	tree::DenseValueTree T = sc.process(um, si->labels);

	auto cutStar = optimalCut(T);
	double vStar = value(T, cutStar.second);
	const double sigmaSquared = vStar*vStar;
	const int seed = 1;

	tree::MCMCTreeCut<tree::DenseValueTree> mcmc(T, sigmaSquared);

//	std::random_device rd;
//	std::default_random_engine re(rd());

	cutStar = mcmc.sample(re, Belief<tree::TreeCut>::SAMPLE_TYPE::MAXIMUM);

	// Validate this cut
	for (size_t n = 0; n < size(T); ++n)
	{
		if (tree::children(T, n).empty())
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
	cv::namedWindow("Crop", cv::WINDOW_NORMAL);

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
//	auto seg = relabelCut(um, T, si->labels, cutStar.second);
//	auto disp = colorByLabel(seg);

//	cv::imshow("Segmentation", disp);
//	cv::imwrite(dir + "cStar.png", disp);

	cv::Mat maskInv = (gt_labels == 0); // All irrelevant pixels
	cv::imshow("Mask", maskInv);

	cv::Mat mask = (gt_labels != 0);
	cv::Mat clusterLabels;
	cv::connectedComponents(mask, clusterLabels, 8, CV_16U);
	cv::namedWindow("Clusters", cv::WINDOW_NORMAL);
	cv::imshow("Clusters", colorByLabel(clusterLabels));

	cv::Mat maskedLabels = cv::Mat::zeros(si->labels.size(), si->labels.type());
	si->labels.copyTo(maskedLabels, mask);
	cv::namedWindow("Masked Labels", cv::WINDOW_NORMAL);
	cv::imshow("Masked Labels", colorByLabel(maskedLabels));

	using LabelT = JaccardMatch::LabelT;
//	JaccardMatch J(clusterLabels, maskedLabels);
	JaccardMatch J(clusterLabels, si->labels);

	// Find leaf nodes that cross clusters
	std::set<LabelT> clusterViolations;
	for (const auto& pair2 : J.lblIndex2.left)
	{
		int j = pair2.second; // Leaf index
		int clusterCount = 0;
		for (const auto& pair1 : J.lblIndex1.left)
		{
			// TODO: Possibly check % of FG/BG pixels
			LabelT clusterLabel = pair1.first;
			if (0 == clusterLabel) { continue; }
			int i = pair1.second; // Cluster index
			if (J.IOU(i, j) > 0)
			{
				++clusterCount;
				if (clusterCount > 1)
				{
					clusterViolations.insert(pair2.first);
					break;
				}
			}
		}
	}

	std::map<LabelT, tree::SparseValueTree> subtrees;
	std::map<LabelT, cv::Rect> subregions;
	for (const auto& pair1 : J.lblIndex1.left)
	{
		LabelT label = pair1.first;
		if (0 == label) { continue; }
		int i = pair1.second;

		AABB bounds;
		std::set<tree::NodeID> leaves;
//		cv::Mat M = cv::Mat::zeros(si->labels.size(), si->labels.type());
		for (const auto& pair2 : J.lblIndex2.left)
		{
			int j = pair2.second;
			if (clusterViolations.find(pair2.first) != clusterViolations.end()) { continue; }
			if (J.intersection(i, j) / static_cast<double>(J.jSizes.at(pair2.first)) > 0.25)
			{
				bounds = merge(bounds, J.boxes2.at(pair2.first));
				leaves.insert(um.label_to_index.at(pair2.first));
//				auto m = (si->labels == pair2.first);
//				M.setTo(label, m);
			}
		}

		auto subtree = tree::extractSubtree(T, leaves);
//		std::cerr << "Tree size: " << size(T) << " -> " << size(subtree) << std::endl;
//		tree::compressTree(subtree);
//		std::cerr << " -> " << size(subtree) << std::endl;
//
//		tree::MCMCTreeCut<tree::SparseValueTree> mcLocal(subtree, sigmaSquared);
//		auto cutStarLocal = mcLocal.sample(re, Belief<tree::TreeCut>::SAMPLE_TYPE::MAXIMUM);
//
//		cv::Mat seg = relabelCut(um, subtree, si->labels, cutStarLocal.second);
//
//		cv::Rect crop(bounds);
//		cv::Mat subregion(si->rgb, crop);
//		cv::imshow("Crop", subregion);
//		cv::imshow("Mask", colorByLabel(M));
//		cv::imshow("Segmentation", colorByLabel(seg));
//		cv::waitKey();
		subtrees.insert({label, subtree});
		subregions.emplace(label, cv::Rect(bounds));
	}
//	subtrees.insert({0, T});

//	cv::waitKey(0);

//	mcmc.sigmaSquared = 1e10*vStar*vStar; // With proposal ratio
//	mcmc.nTrials = 100;
	mcmc.sigmaSquared = 0.5*vStar*vStar; // The larger this number, the more permissive exploration is allowed
	mcmc.nTrials = 50;

	const int nSamples = 100;
	for (const auto& pair : subtrees)
	{
		LabelT subProblem = pair.first;
		const auto& subTree = pair.second;

		cv::Mat subLabels(si->labels, subregions.at(subProblem));
		cv::Mat subLabelGT(gt_labels, subregions.at(subProblem));

		tree::MCMCTreeCut<tree::SparseValueTree> mcLocal(subTree, sigmaSquared);
		cutStar = mcLocal.sample(re, Belief<tree::TreeCut>::SAMPLE_TYPE::MAXIMUM);

		std::string subDir = dir + "/" + std::to_string(subProblem) + "/";
		fs::create_directory(subDir);
		std::ofstream csv;
		csv.open(subDir + "data.csv", std::ios::out);

		csv << "Trial\tlogp(cut)\tJaccard\tCover" << std::endl;

		for (int i = 0; i < nSamples; ++i)
		{
			std::pair<double, tree::TreeCut> cut;
			if (0 == i)
			{
				cut = cutStar;
			}
			else
			{
				cut = mcLocal.sample(re, Belief<tree::TreeCut>::SAMPLE_TYPE::RANDOM);
			}

			cv::Mat seg = relabelCut(um, subTree, subLabels, cut.second);
//			seg.setTo(0, maskInv);
			JaccardMatch jaccard(subLabelGT, seg);
			Colormap cmap = colormap;
			extendColormap(cmap, seg, re);
			cv::Mat disp = colorByLabel(relabel(seg, jaccard.match.second.right), cmap);
			cv::imshow("Segmentation", disp);
			cv::imwrite(subDir + "c" + std::to_string(i) + ".png", disp);
			csv << i << "\t" << cut.first << "\t" << jaccard.match.first << "\t" << jaccard.symmetricCover() << std::endl;

//		cv::waitKey(10);
		}
	}

//	csv.close();

//	cv::waitKey(0);
	return 0;

}