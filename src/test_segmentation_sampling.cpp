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

#include "mps_voxels/DataLog.h"

#include "mps_voxels/image_utils.h"
#include <opencv2/highgui.hpp>

using namespace mps;


std::string cvType2Str(int type)
{
	std::string r;
	switch (type)
	{
	case CV_8UC1:  return sensor_msgs::image_encodings::MONO8;
	case CV_8UC3:  return sensor_msgs::image_encodings::BGR8;
	case CV_16UC1: return sensor_msgs::image_encodings::MONO16;
	default: ; // Do nothing
	}

	uchar depth = type & CV_MAT_DEPTH_MASK;
	uchar chans = 1 + (type >> CV_CN_SHIFT);

	switch ( depth ) {
	case CV_8U:  r = "8U"; break;
	case CV_8S:  r = "8S"; break;
	case CV_16U: r = "16U"; break;
	case CV_16S: r = "16S"; break;
	case CV_32S: r = "32S"; break;
	case CV_32F: r = "32F"; break;
	case CV_64F: r = "64F"; break;
	default:     r = "User"; break;
	}

	r += "C";
	r += (chans+'0');

	return r;
}


#include <boost/preprocessor/seq/enum.hpp>
#include <boost/preprocessor/seq/for_each.hpp>
#include <boost/preprocessor/variadic/to_seq.hpp>

/**
 * @def STRINGIFY STRINGIFY(kitten) resolves to "kitten" in the post-processed source
 */
#define STRINGIFY2(X) #X
#define STRINGIFY(X) STRINGIFY2(X)

// Run the given macro on all the variadic elements passed
#define ID_OP(_, func, elem) func(elem)
#define APPLY_TO_ALL(func, ...)                \
    BOOST_PP_SEQ_FOR_EACH(                     \
        ID_OP, func,                           \
        BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__)  \
    )

template <typename T>
struct ros_message_conversion {};

template <typename T>
struct ros_message_type_of {};

/**
 * @tparam T Main type to be converted to message
 * @tparam Aux Auxiliary type containing any extra info (particularly headers)
 * @param t Object to convert
 * @param a Aux data object (defaults to null pointer)
 * @return
 */
template <typename T, typename Aux = nullptr_t>
auto toMessage(T t, Aux a = nullptr)
{
	return ros_message_conversion<std::remove_cv_t<std::remove_reference_t<T>>>::toMessage(t, a);
}

template <typename T>
auto fromMessage(T t)
{
	return ros_message_conversion<typename ros_message_type_of<std::remove_cv_t<std::remove_reference_t<T>>>::CppType>::fromMessage(t);
}


template<>
struct ros_message_type_of<sensor_msgs::Image> { using CppType = cv::Mat; };

template<>
struct ros_message_conversion<cv::Mat>
{
	using MsgType = sensor_msgs::Image;
	using CppType = cv::Mat;

	static
	MsgType toMessage(const CppType& t, const nullptr_t)
	{
		return *cv_bridge::CvImage(std_msgs::Header(), cvType2Str(t.type()), t).toImageMsg();
	}

	static
	MsgType toMessage(const CppType& t, const std_msgs::Header& h)
	{
		return *cv_bridge::CvImage(h, cvType2Str(t.type()), t).toImageMsg();
	}

	static
	CppType fromMessage(const MsgType& t)
	{
		return cv_bridge::toCvCopy(t, t.encoding)->image;
	}
};


#define LOG_IMAGE_MESSAGE(var_name) \
	activeChannels.insert(channel + "/" STRINGIFY(var_name) ); \
	log(channel + "/" STRINGIFY(var_name) , toMessage(msg.var_name, header));
template <>
void DataLog::log<SegmentationInfo>(const std::string& channel, const SegmentationInfo& msg)
{
	std_msgs::Header header; header.stamp = msg.t; header.frame_id = msg.frame_id;
	APPLY_TO_ALL(LOG_IMAGE_MESSAGE, rgb, depth, ucm2, labels2, centroids2, stats2, display_contours, labels)

	activeChannels.insert(channel + "/objectness_segmentation"); \
	log(channel + "/objectness_segmentation", *msg.objectness_segmentation->toImageMsg());
}

#define LOAD_IMAGE_MESSAGE(var_name) \
	load(channel + "/" STRINGIFY(var_name) , im); msg.var_name = fromMessage(im);
template <>
bool DataLog::load<SegmentationInfo>(const std::string& channel, SegmentationInfo& msg)
{
	sensor_msgs::Image im;
	APPLY_TO_ALL(LOAD_IMAGE_MESSAGE, rgb, depth, ucm2, labels2, centroids2, stats2, display_contours, labels)
	msg.t = im.header.stamp; msg.frame_id = im.header.frame_id;

	load(channel + "/objectness_segmentation" , im);
	msg.objectness_segmentation = cv_bridge::toCvCopy(im);
	return true;
}

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

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "segmentation_sampling");
	ros::NodeHandle nh, pnh("~");

	auto si = std::make_shared<SegmentationInfo>();

	std::shared_ptr<DataLog> log;
	try
	{
		log = std::make_shared<DataLog>("", std::unordered_set<std::string>{"segmentation"}, rosbag::BagMode::Read);
		if (!log->load("segmentation", *si))
		{
			throw std::runtime_error("");
		}
	}
	catch (...)
	{
		// TODO: Make loadable from file
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
		if (historian->buffer.rgb.empty()) { throw std::runtime_error("Failed to get any data."); }
		const SensorHistoryBuffer& buff = historian->buffer;

		auto segmentationClient = std::make_shared<RGBDSegmenter>(nh);
		si = segmentationClient->segment(*buff.rgb.begin()->second, *buff.depth.begin()->second, buff.cameraModel.cameraInfo());
		if (!si) { throw std::runtime_error("Failed to get segmentation."); }

		log = std::make_shared<DataLog>("", std::unordered_set<std::string>{"segmentation"}, rosbag::BagMode::Write);
		log->log("segmentation", *si);
	}


	Ultrametric um(si->ucm2, si->labels2);
	SceneCut sc;
	ValueTree T = sc.process(um, si->labels);

	const double sigmaSquared = 10.0;

	MCMCTreeCut mcmc(T, sigmaSquared);

	std::random_device rd;
	std::default_random_engine re(rd());

	auto cutStar = mcmc.sample(re, Belief<TreeCut>::SAMPLE_TYPE::MAXIMUM);

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

	cv::imshow("Image", si->rgb);
	cv::imshow("Labels", colorByLabel(si->labels));
	cv::imshow("Contours", si->display_contours);
	cv::imshow("Objectness", colorByLabel(si->objectness_segmentation->image));

	std::cerr << "# original objects: " << unique(si->objectness_segmentation->image).size() << std::endl;
	std::cerr << "# new objects: " << cutStar.second.size() << std::endl;

	auto seg = relabelCut(um, T, si->labels, cutStar.second);
	auto disp = colorByLabel(seg);


	cv::imshow("Segmentation", disp);

	cv::waitKey(0);

	return 0;

}