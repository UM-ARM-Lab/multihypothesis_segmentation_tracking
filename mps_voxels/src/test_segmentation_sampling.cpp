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
#include "mps_voxels/SegmentationTreeSampler.h"
#include "mps_voxels/JaccardMatch.h"
#include "mps_voxels/ValueTree_impl.hpp"
#include "mps_voxels/relabel_tree_image.hpp"
#include "mps_voxels/colormap.h"

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

//template <typename T>
//using DataGeneratorFn = std::function<bool(T&)>;

using LabelT = JaccardMatch::LabelT;
using ExtractionMap = boost::bimap<tree::NodeID, tree::NodeID>;
using CorrespondenceMap = boost::bimap<tree::NodeID, LabelT>;
using CorrespondenceTree = std::pair<tree::DenseValueTree, CorrespondenceMap>;

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
			data = log->load<T>(channelName);
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

std::string rgb2hex(int r, int g, int b, bool with_head)
{
	std::stringstream ss;
	if (with_head)
		ss << "#";
	ss << std::hex << (r << 16 | g << 8 | b );
	return ss.str();
}

template <typename T>
void printGraphviz(std::ostream& os, const T& t)
{
	os << "digraph g {" << std::endl;
	os << "rankdir=\"BT\";" << std::endl;
	for (size_t n = 0; n < size(t); ++n)
	{
		os << n << "->" << tree::parent(t, n) << ";";
	}
	os << "}" << std::endl;
}

template <typename T>
void printGraphviz(std::ostream& os, const T& t, const ExtractionMap& sparseToDenseID)
{
	os << "digraph g {" << std::endl;
	os << "rankdir=\"BT\";" << std::endl;
	for (size_t n = 0; n < size(t); ++n)
	{
		os << sparseToDenseID.right.at(n) << "->" << sparseToDenseID.right.at(tree::parent(t, n)) << ";";
	}
	os << std::endl << "}" << std::endl;
}

template <typename T>
void printGraphviz(std::ostream& os, const T& t, const tree::TreeCut& cut)
{
	os << "digraph g {" << std::endl;
	os << "rankdir=\"BT\";" << std::endl;
	Eigen::Map<const Eigen::VectorXd> dataMap(t.value_.data(), size(t));
	double minVal = dataMap.minCoeff();
	double maxVal = dataMap.maxCoeff();
	for (size_t n = 0; n < size(t); ++n)
	{
		if (cut.find(n) != cut.end())
		{
			os << n << R"([style="filled" fillcolor="red"];)";
		}
		else
		{
			double normedVal = (value(t, n) - minVal) / (maxVal - minVal);
			assert(0 <= normedVal);
			assert(normedVal <= 1.0);

			double r, g, b;
			colormap(igl::parula_cm, std::pow(normedVal, 8), r, g, b);
			std::string colorKey = rgb2hex(r*255, g*255, b*255, true);
			os << n << R"([style="filled" fillcolor=")" << colorKey << R"("];)";
		}
	}

	for (size_t n = 0; n < size(t); ++n)
	{
		os << n << "->" << tree::parent(t, n) << ";";
	}
	os << "}" << std::endl;
}

std::ostream& operator<<(std::ostream& os, const tree::TreeCut& t)
{
	for (const auto& node : t)
	{
		os << std::setw(3) << node << " ";
	}
	return os;
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
		gt_labels = log->load<cv::Mat>("ground_truth/labels");
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

//	std::random_device rd;
	std::default_random_engine re(0);// (rd());
	Colormap colormap = createColormap(gt_labels, re);

	SegmentationTreeSampler treeSampler(si);

	printGraphviz(std::cerr, treeSampler.vt);

//	Ultrametric um(si->ucm2, si->labels2);
//	SceneCut sc;
//	tree::DenseValueTree T = sc.process(um, si->labels);
//
//	auto cutStar = optimalCut(T);
//	double vStar = value(T, cutStar.second);
//	const double sigmaSquared = vStar*vStar;
//	const int seed = 1;
//
//	tree::MCMCTreeCut<tree::DenseValueTree> mcmc(T, sigmaSquared);

#ifndef NDEBUG
	// Validate this cut
	auto cutStar = treeSampler.sample(re, SAMPLE_TYPE::MAXIMUM);
	for (size_t n = 0; n < size(treeSampler.vt); ++n)
	{
		if (tree::children(treeSampler.vt, n).empty())
		{
			auto an = ancestors(treeSampler.vt, n);
			an.insert(n);
			std::vector<int> crossover;
			std::set_intersection(an.begin(), an.end(), treeSampler.cutStar.begin(), treeSampler.cutStar.end(), std::back_inserter(crossover));
			assert(crossover.size() == 1); // A leaf node should have exactly one leaf node that is a parent.
		}
	}
#endif

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
	cv::imshow("Objectness", colorByLabel(si->objectness_segmentation->image));

	std::string dir = "/tmp/segmentation/";
	cv::imwrite(dir + "image.png", si->rgb);
	cv::imwrite(dir + "contours.png", si->display_contours);
	cv::imwrite(dir + "objectness.png", colorByLabel(si->objectness_segmentation->image));
	cv::imwrite(dir + "ground.png", colorByLabel(gt_labels, colormap));

	std::cerr << "# original objects: " << unique(si->objectness_segmentation->image).size() << std::endl;
	std::cerr << "# new objects: " << treeSampler.cutStar.size() << std::endl;

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

//	JaccardMatch J(clusterLabels, maskedLabels);
	JaccardMatch J(clusterLabels, si->labels);

	// Find leaf nodes that cross clusters
//	std::set<LabelT> clusterViolations;
//	for (const auto& pair2 : J.lblIndex2.left)
//	{
//		int j = pair2.second; // Leaf index
//		int clusterCount = 0;
//		for (const auto& pair1 : J.lblIndex1.left)
//		{
//			// TODO: Possibly check % of FG/BG pixels
//			LabelT clusterLabel = pair1.first;
//			if (0 == clusterLabel) { continue; }
//			int i = pair1.second; // Cluster index
//			if (J.IOU(i, j) > 0)
//			{
//				++clusterCount;
//				if (clusterCount > 1)
//				{
//					clusterViolations.insert(pair2.first);
//					break;
//				}
//			}
//		}
//	}

	CorrespondenceMap treeIndexToImageLabel;
	for (const auto& pair : treeSampler.um->label_to_index)
	{
		treeIndexToImageLabel.insert({static_cast<tree::NodeID>(pair.second), static_cast<LabelT>(pair.first)});
	}


	std::map<SubproblemIndex, CorrespondenceTree> subtrees;
	std::map<SubproblemIndex, cv::Rect> subregions;
	std::map<SubproblemIndex, ExtractionMap> extractions;
	for (const auto& pair1 : J.lblIndex1.left)
	{
		SubproblemIndex label; label.id = pair1.first;
		if (0 == label.id) { continue; }
		int i = pair1.second;

		AABB bounds;
		std::set<tree::NodeID> leaves;
//		cv::Mat M = cv::Mat::zeros(si->labels.size(), si->labels.type());
		for (const auto& pair2 : J.lblIndex2.left)
		{
			int j = pair2.second;
//			if (clusterViolations.find(pair2.first) != clusterViolations.end()) { continue; }
			if (J.intersection(i, j) / static_cast<double>(J.jSizes.at(pair2.first)) > 0.5)
			{
				bounds = merge(bounds, J.boxes2.at(pair2.first));
				leaves.insert(treeSampler.um->label_to_index.at(pair2.first));
//				auto m = (si->labels == pair2.first);
//				M.setTo(label, m);
			}
		}

		auto subtree = tree::extractSubtree(treeSampler.vt, leaves);
		// Compress
		// Densify
		auto dense = tree::densify(subtree);
		CorrespondenceMap subtreeIndexToImageLabel;
		for (const auto& pair : dense.second.left)
		{
			subtreeIndexToImageLabel.insert({pair.second, treeIndexToImageLabel.left.at(pair.first)});
		}
		subtrees.insert({label, {dense.first, subtreeIndexToImageLabel}});
		subregions.emplace(label, cv::Rect(bounds));
		extractions.emplace(label, dense.second);
//		std::cerr << "Tree size: " << size(T) << " -> " << size(subtree) << std::endl;
//		tree::compressTree(subtree);
//		std::cerr << " -> " << size(subtree) << std::endl;
//
//		tree::MCMCTreeCut<tree::SparseValueTree> mcLocal(subtree, sigmaSquared);
//		auto cutStarLocal = mcLocal.sample(re, SAMPLE_TYPE::MAXIMUM);
//
//		cv::Mat seg = relabelCut(um, subtree, si->labels, cutStarLocal.second);
//		cv::Mat seg = relabelCut(dense.first, cut.second, subTree.second.left, subLabels);
//
//		cv::Rect crop(bounds);
//		cv::Mat subregion(si->rgb, crop);
//		cv::imshow("Crop", subregion);
//		cv::imshow("Mask", colorByLabel(M));
//		cv::imshow("Segmentation", colorByLabel(seg));
//		cv::waitKey();
	}
//	subtrees.insert({0, T});

//	cv::waitKey(0);

//	mcmc.sigmaSquared = 1e10*vStar*vStar; // With proposal ratio
//	mcmc.nTrials = 100;
//	mcmc.sigmaSquared = 0.5*vStar*vStar; // The larger this number, the more permissive exploration is allowed
//	mcmc.nTrials = 50;

//	const int maxRejections = 100;
	const int nSamples = 5000;
	for (const auto& pair : subtrees)
	{
		SubproblemIndex subProblem = pair.first;
		const auto& subTree = pair.second;

		cv::Mat subLabels(si->labels, subregions.at(subProblem));
		cv::Mat subLabelGT(gt_labels, subregions.at(subProblem));

		auto cutStarLocal = tree::optimalCut(subTree.first);
		tree::MCMCTreeCut<tree::DenseValueTree> mcLocal(subTree.first, 0.5 * cutStarLocal.first * cutStarLocal.first);

//		printGraphviz(std::cerr, subTree.first);
		printGraphviz(std::cerr, subTree.first, extractions.at(subProblem));

		fs::path subDir(dir + "/" + std::to_string(subProblem.id) + "/");
		fs::create_directories(subDir);
		std::ofstream csv;
		csv.open(subDir.string() + "data.csv", std::ios::out);

//		for (size_t l = 0; l < size(subTree.first); ++l)
//		{
//			tree::NodeID n = size(subTree.first) - l - 1;
//			tree::TreeCut singletonCut = {static_cast<int>(n)};
////			int label = subTree.second.left.at(l);
//			cv::Mat singleSeg = relabelCut(subTree.first, singletonCut, subTree.second.left, subLabels);
//
//			std::ofstream dot;
//			dot.open(subDir.string() + "data.dot", std::ios::out);
//			printGraphviz(dot, subTree.first, singletonCut);
//			dot.close();
//
//			cv::imshow("Segmentation", colorByLabel(singleSeg));
//			cv::waitKey(0);
//		}


		csv << "Trial,logp(cut),Jaccard,Cover,cut" << std::endl;

//		int numRejections = 0;
		std::set<tree::TreeCut> cuts;
		for (int i = 0; i < nSamples; ++i)
		{
			std::pair<double, tree::TreeCut> cut;
			if (0 == i)
			{
				cut = cutStarLocal;
				cut.first = 0.0;
			}
			else
			{
				cut = mcLocal.sample(re, SAMPLE_TYPE::RANDOM);
			}

//			const auto& cutPair = cuts.insert(cut.second);
//			if (!cutPair.second)
//			{
//				// Already have this one: try again
//				std::cerr << "Rejected duplicate cut " << cut.second << std::endl;
//				++numRejections;
//				if (numRejections < maxRejections)
//				{
//					--i;
//					continue;
//				}
//			}

//			{
//				std::ofstream dot;
//				dot.open(subDir.string() + "data.dot", std::ios::out);
//				printGraphviz(dot, subTree.first, cut.second);
//				dot.close();
//			}

			cv::Mat seg = relabelCut(subTree.first, cut.second, subTree.second.left, subLabels);
//			seg.setTo(0, maskInv);
			JaccardMatch jaccard(subLabelGT, seg);
			Colormap cmap = colormap;
			extendColormap(cmap, seg, re);
			cv::Mat disp = colorByLabel(relabel(seg, jaccard.match.second.right), cmap);
//			cv::imshow("Segmentation", disp);
//			cv::imwrite(subDir.string() + "c" + std::to_string(i) + ".png", disp);
			csv.setf(std::ios::fixed, std::ios::floatfield);
			csv << std::setw(3) << i << ","
			    << std::setw(8) << std::setprecision(5) << cut.first << ","
				<< std::setw(8) << std::setprecision(5) << jaccard.match.first << ","
				<< std::setw(8) << std::setprecision(5) << jaccard.symmetricCover() << ","
			    << "{" << cut.second << "}" << std::endl;

//			{
//				std::ofstream dot;
//				dot.open(subDir.string() + std::to_string(i) + ".dot", std::ios::out);
//				printGraphviz(dot, subTree.first, cut.second);
//				dot.close();
//			}

//			cv::waitKey(0);
		}
		return 0;
	}

//	csv.close();

//	cv::waitKey(0);
	return 0;

}