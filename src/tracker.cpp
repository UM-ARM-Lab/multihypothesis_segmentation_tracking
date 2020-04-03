//
// Created by arprice on 8/13/18.
//

//#include <opencv2/imgproc.hpp>
//#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
//
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>
//#include <depth_image_proc/depth_conversions.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

#include "mps_voxels/util/assert.h"
#include "mps_voxels/CudaTracker.h"
#include "mps_voxels/Scene.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/video_graph.h"
#include "mps_voxels/Ultrametric.h"
#include "mps_voxels/graph_matrix_utils.h"
#include "mps_voxels/LocalOctreeServer.h"
#include "mps_voxels/OctreeMotionModel.h"

#include <mps_msgs/SegmentGraph.h>
#include <mps_msgs/ClusterRigidMotionsAction.h>
#include <mps_msgs/MatchFrameSegmentsAction.h>

#include <actionlib/client/simple_action_client.h>
#include <visualization_msgs/MarkerArray.h>
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/SparseCore>

#include <ros/ros.h>

using RigidClusterClient = actionlib::SimpleActionClient<mps_msgs::ClusterRigidMotionsAction>;
using FrameMatchClient = actionlib::SimpleActionClient<mps_msgs::MatchFrameSegmentsAction>;

std::unique_ptr<Tracker> tracker;
std::shared_ptr<RGBDSegmenter> segmentationClient;
std::unique_ptr<image_transport::Publisher> segmentationPub;

double SIFT_MATCH_WEIGHT = 1.0/5.0;
double RIGID_MATCH_WEIGHT = 1.0/5.0;
int NUM_OUTPUT_LABELS = 15;
//int pyrScale = 2;

mps_msgs::Segmentation toSegmentationMsg(const SegmentationInfo& si)
{
	mps_msgs::Segmentation s;
	sensor_msgs::CameraInfo info = tracker->cameraModel.cameraInfo();
	info.header.stamp = si.t;
	s.camera_info = info;
	s.rgb = *cv_bridge::CvImage(info.header, sensor_msgs::image_encodings::BGR8, si.rgb).toImageMsg();
	s.depth = *cv_bridge::CvImage(info.header, sensor_msgs::image_encodings::TYPE_16UC1, si.depth).toImageMsg();
//	s.segments = *cv_bridge::CvImage(info.header, sensor_msgs::image_encodings::MONO16, si.labels).toImageMsg();
	s.segments = *cv_bridge::CvImage(info.header, sensor_msgs::image_encodings::MONO16, si.objectness_segmentation->image).toImageMsg();
//	s.ucm = *cv_bridge::CvImage(info.header, sensor_msgs::image_encodings::TYPE_64FC1, si.ucm).toImageMsg();
	return s;
}

template <typename T>
void getSetIfMissing(ros::NodeHandle& nh, const std::string& param_name, T& param_val)
{
	if (!nh.hasParam(param_name))
	{
		nh.setParam(param_name, param_val);
	}
	else
	{
		nh.getParam(param_name, param_val);
	}
}
/*
static void drawOptFlowMap(const cv::Mat& flow, cv::Mat& cflowmap, int step,
                           double, const cv::Scalar& color)
{
	for(int y = 0; y < cflowmap.rows; y += step)
		for(int x = 0; x < cflowmap.cols; x += step)
		{
			const cv::Point2f& fxy = flow.at<cv::Point2f>(y, x);
			cv::line(cflowmap, cv::Point(x,y), cv::Point(cvRound(x+fxy.x), cvRound(y+fxy.y)),
			     color);
			cv::circle(cflowmap, cv::Point(x,y), 2, color, -1);
		}
}
*/
/*
void track()
{
	cv::Mat flow, cflow, display;
	cv::UMat gray1, gray2, uflow; // Possibly CPU or GPU image matrix
	cv::cvtColor(rgb_buffer.front()->image, gray1, cv::COLOR_BGR2GRAY);
	for (int i = 1; i < static_cast<int>(rgb_buffer.size()) && ros::ok(); ++i)
	{
		rgb_buffer[i-1]->image.copyTo(display);
		cv::cvtColor(rgb_buffer[i]->image, gray2, cv::COLOR_BGR2GRAY);

//		cv::calcOpticalFlowFarneback(gray1, gray2, uflow, 0.4, 3, 12, 5, 7, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
//		cv::calcOpticalFlowFarneback(gray1, gray2, uflow, 0.4, 1, 12, 5, 7, 1.5, cv::OPTFLOW_FARNEBACK_GAUSSIAN);
//		uflow.copyTo(flow);
		cv::calcOpticalFlowFarneback(gray1, gray2, uflow, 0.3, 4, 5, 15, 5, 1.2, cv::OPTFLOW_FARNEBACK_GAUSSIAN);

		const int GRID_STEP = 15;

		cv::cvtColor(gray1, cflow, cv::COLOR_GRAY2BGR);
		uflow.copyTo(flow);
		drawOptFlowMap(flow, cflow, 16, 1.5, cv::Scalar(0, 255, 0));

//		// By y += 5, x += 5 you can specify the grid
//		for (int y = 0; y < display.rows; y += GRID_STEP)
//		{
//			for (int x = 0; x < display.cols; x += GRID_STEP)
//			{
//				// get the flow from y, x position * 10 for better visibility
//				const cv::Point2f flowatxy = flow.at<cv::Point2f>(y, x) * 10;
//				// draw line at flow direction
//				cv::line(display, cv::Point(x, y), cv::Point(cvRound(x + flowatxy.x), cvRound(y + flowatxy.y)), cv::Scalar(255,0,0));
//				// draw initial point
//				cv::circle(display, cv::Point(x, y), 1, cv::Scalar(0, 0, 0), -1);
//			}
//		}

//		cv::imshow("prev", display);
		cv::imshow("prev", cflow);
		cv::waitKey(100);

		std::swap(gray1, gray2);
	}
}
*/

void decomposeAdjoint(const Eigen::MatrixXd& M, Eigen::VectorXd& eigenvalues, Eigen::MatrixXd& eigenvectors)
{
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver;
	solver.compute(M, Eigen::ComputeEigenvectors);
	eigenvalues = solver.eigenvalues();
	eigenvectors = solver.eigenvectors();
}

// Signum function
template <typename T> int sgn(T val) {
	return (T(0) < val) - (val < T(0));
}

//void cutGraph(VideoSegmentationGraph& graph, const std::vector<size_t>& cuts, const Eigen::MatrixXd& eigenvectors)
//{
//	std::set<VideoSegmentationGraph::edge_descriptor> cutEdges;
//	for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::edges(graph)))
//	{
//		VideoSegmentationGraph::vertex_descriptor u = boost::source(ed, graph);
//		VideoSegmentationGraph::vertex_descriptor v = boost::target(ed, graph);
//
//		for (const size_t c : cuts)
//		{
//			if (sgn(eigenvectors.col(c)[u]) != sgn(eigenvectors.col(c)[v]))
//			{
//				cutEdges.insert(ed);
//				break;
//			}
//		}
//	}
//
//	for (VideoSegmentationGraph::edge_descriptor ed : cutEdges)
//	{
//		boost::remove_edge(ed, graph);
//	}
//}

std::vector<cv::Point2i> getNeighbors(const int row, const int col, const cv::Mat& img)
{
	std::vector<cv::Point2i> neighbors; neighbors.reserve(8);
	std::vector<int> rows, cols; rows.reserve(3); cols.reserve(3);
	// Enumerate rows to search
	if (row > 0) { rows.push_back(row-1); }
	rows.push_back(row);
	if (row < img.rows-1) { rows.push_back(row+1); }

	// Enumerate cols to search
	if (col > 0) { cols.push_back(col-1); }
	cols.push_back(col);
	if (col < img.cols-1) { cols.push_back(col+1); }

	for (const int r : rows)
	{
		for (const int c : cols)
		{
			if (r == row && c == col)
			{
				continue;
			}
			neighbors.emplace_back(cv::Point2i(c, r));
		}
	}

	return neighbors;
}

std::set<std::pair<uint16_t, uint16_t>> computeNeighbors(const cv::Mat& labels)
{
	std::set<std::pair<uint16_t, uint16_t>> neighbors;
	for(int i = 0; i < labels.rows; i++)
	{
		for(int j = 0; j < labels.cols; j++)
		{
			const uint16_t val = labels.at<uint16_t>(i, j);
			if (0 == val)
			{
				std::set<uint16_t> neighborhood;
				for (const auto& pt : getNeighbors(i, j, labels))
				{
					const uint16_t nval = labels.at<uint16_t>(pt);
					if (0 != nval)
					{
						neighborhood.insert(nval);
					}
				}
				for (const auto a : neighborhood)
				{
					for (const auto b : neighborhood)
					{
						if (a != b)
						{
							neighbors.insert({a, b});
						}
					}
				}
			}
		}
	}
	return neighbors;
}

std::map<std::pair<uint16_t, uint16_t>, double> computeUcmDual(const cv::Mat& labels, const cv::Mat& ucm)
{
	MPS_ASSERT(labels.size == ucm.size);
	std::map<std::pair<uint16_t, uint16_t>, double> neighbors;
	for(int i = 0; i < labels.rows; i++)
	{
		for(int j = 0; j < labels.cols; j++)
		{
			const uint16_t val = labels.at<uint16_t>(i, j);
			if (0 == val)
			{
				if (0 < i && i < labels.rows-1)
				{
					const uint16_t llabel = labels.at<uint16_t>(i-1, j);
					const uint16_t rlabel = labels.at<uint16_t>(i+1, j);
					if (llabel != 0 && rlabel != 0 && llabel != rlabel)
					{
						std::pair<uint16_t, uint16_t> p(std::min(llabel, rlabel), std::max(llabel, rlabel));
						double edge_val = ucm.at<double>(i, j);
						edge_val = edge_val * edge_val;

						auto iter = neighbors.find(p);
						if (iter == neighbors.end())
						{
							neighbors.emplace(p, edge_val);
						}
						else
						{
							iter->second = std::min(edge_val, iter->second);
						}
					}
				}

				if (0 < j && j < labels.cols-1)
				{
					const uint16_t blabel = labels.at<uint16_t>(i, j-1);
					const uint16_t tlabel = labels.at<uint16_t>(i, j+1);
					if (blabel != 0 && tlabel != 0 && blabel != tlabel)
					{
						std::pair<uint16_t, uint16_t> p(std::min(blabel, tlabel), std::max(blabel, tlabel));
						double edge_val = ucm.at<double>(i, j);
						edge_val = edge_val * edge_val;

						auto iter = neighbors.find(p);
						if (iter == neighbors.end())
						{
							neighbors.emplace(p, edge_val);
						}
						else
						{
							iter->second = std::min(edge_val, iter->second);
						}
					}
				}
			}
		}
	}
	return neighbors;
}

#define REQUIRE_OBJECTNESS_IN_GRAPH true

void addToGraph(VideoSegmentationGraph<SEGMENT_TYPE::UCM>& G, const SegmentationInfo& si)
{
	const cv::Mat& ucm = si.ucm2;
	const cv::Mat& full_labels = si.labels2;
	const ros::Time k = si.t;

	// Frame index, label value
	SegmentLookup<SEGMENT_TYPE::UCM> segmentToNode;

	for (const auto lbl : unique(full_labels))
	{
		if (0 == lbl) { continue; }
#if REQUIRE_OBJECTNESS_IN_GRAPH
		if (cv::countNonZero((0 != si.objectness_segmentation->image) & (lbl == si.labels)) < cv::countNonZero(lbl == si.labels)/4) { continue; }
#endif

		VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor v = boost::add_vertex(NodeProperties<SEGMENT_TYPE::UCM>({k, lbl}), G);
		segmentToNode.insert({{k, lbl}, v});
	}

	auto neighbor_pairs = computeUcmDual(full_labels, ucm);
	for (const auto edge : neighbor_pairs)
	{
		label_type p = edge.first.first;
		label_type q = edge.first.second;
		double dist = edge.second;
		if (dist <= 0 || 1.0 < dist) { throw std::runtime_error("Ultrametric max distance should be <= 1.0"); }

		VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor s, t;
		const auto a = segmentToNode.find({k, p});
		const auto b = segmentToNode.find({k, q});

#if REQUIRE_OBJECTNESS_IN_GRAPH
		if (a == segmentToNode.end() || b == segmentToNode.end()) { continue; }
#else
		MPS_ASSERT(a != segmentToNode.end());
		MPS_ASSERT(b != segmentToNode.end());
#endif

		s = a->second;
		t = b->second;

		boost::add_edge(s, t, {1.0/(dist+1e-4)}, G);
	}

#if !REQUIRE_OBJECTNESS_IN_GRAPH
	for (VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor vd : make_range(boost::vertices(G)))
	{
		MPS_ASSERT(boost::out_degree(vd, G) > 0);
	}
#endif
}

void joinFrameGraphs(VideoSegmentationGraph<SEGMENT_TYPE::UCM>& G, const SegmentationInfo& iInfo, const SegmentationInfo& jInfo, const Tracker::Flow2D& flow)
{
	SegmentLookup<SEGMENT_TYPE::UCM> segmentToNode;
	for (VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor vd : make_range(boost::vertices(G)))
	{
		const NodeProperties<SEGMENT_TYPE::UCM>& np = G[vd];
		segmentToNode[{np.t, np.leafID}] = vd;
	}

	const ros::Time& i = iInfo.t;
	const ros::Time& j = jInfo.t;
	if (i == j) throw std::logic_error("Sad!");

	const cv::Mat& iLabels = iInfo.labels;
	const cv::Mat& jLabels = jInfo.labels;

	for (std::pair<cv::Point2f, cv::Point2f> flowVec : flow)
	{
		// NB: Flow may be computed for the full image, so flow points may be outside the ROI
		if (!iInfo.roi.contains(cv::Point2i(flowVec.first.x, flowVec.first.y))) { continue; }
		if (!jInfo.roi.contains(cv::Point2i(flowVec.second.x, flowVec.second.y))) { continue; }

		flowVec.first.x -= iInfo.roi.x;
		flowVec.first.y -= iInfo.roi.y;
		flowVec.second.x -= jInfo.roi.x;
		flowVec.second.y -= jInfo.roi.y;

		cv::Point2i iPt(flowVec.first.x, flowVec.first.y);
		cv::Point2i jPt(flowVec.second.x, flowVec.second.y);

//		if (iPt.x < 0 || iPt.y < 0) { continue; }
//		if (iPt.x >= iLabels.cols || iPt.y >= iLabels.rows) { continue; }
//		if (jPt.x < 0 || jPt.y < 0) { continue; }
//		if (jPt.x >= jLabels.cols || jPt.y >= jLabels.rows) { continue; }

		int iLabel = iLabels.at<uint16_t>(iPt);
		int jLabel = jLabels.at<uint16_t>(jPt);

		if (iLabel == 0 || jLabel == 0)
		{
			continue;
		}

		auto a = segmentToNode.find({i, iLabel});
		auto b = segmentToNode.find({j, jLabel});

#if REQUIRE_OBJECTNESS_IN_GRAPH
		if (a == segmentToNode.end() || b == segmentToNode.end()) { continue; }
#else
		MPS_ASSERT(a != segmentToNode.end());
		MPS_ASSERT(b != segmentToNode.end());
#endif

		auto res = boost::add_edge(a->second, b->second, G);
		EdgeProperties& eProps = G[res.first];
		if (eProps.affinity == 0) { eProps.affinity = 1.0; }
		eProps.affinity += SIFT_MATCH_WEIGHT;
	}
}

void joinMotionCliques(VideoSegmentationGraph<SEGMENT_TYPE::UCM>& G, const SegmentationInfo& iInfo, const SegmentationInfo& jInfo, const Tracker::Flow2D& flow)
{
	SegmentLookup<SEGMENT_TYPE::UCM> segmentToNode;
	for (VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor vd : make_range(boost::vertices(G)))
	{
		const NodeProperties<SEGMENT_TYPE::UCM>& np = G[vd];
		segmentToNode[{np.t, np.leafID}] = vd;
	}

	const ros::Time& i = iInfo.t;
	const ros::Time& j = jInfo.t;

	const cv::Mat& iLabels = iInfo.labels;
	const cv::Mat& jLabels = jInfo.labels;

	std::set<uint16_t> iSet, jSet;

	for (std::pair<cv::Point2f, cv::Point2f> flowVec : flow)
	{
		// NB: Flow may be computed for the full image, so flow points may be outside the ROI
		if (!iInfo.roi.contains(cv::Point2i(flowVec.first.x, flowVec.first.y))) { continue; }
		if (!jInfo.roi.contains(cv::Point2i(flowVec.second.x, flowVec.second.y))) { continue; }

		flowVec.first.x -= iInfo.roi.x;
		flowVec.first.y -= iInfo.roi.y;
		flowVec.second.x -= jInfo.roi.x;
		flowVec.second.y -= jInfo.roi.y;

		cv::Point2i iPt(flowVec.first.x, flowVec.first.y);
		cv::Point2i jPt(flowVec.second.x, flowVec.second.y);

		uint16_t iLabel = iLabels.at<uint16_t>(iPt);
		uint16_t jLabel = jLabels.at<uint16_t>(jPt);

		if (iLabel == 0 || jLabel == 0)
		{
			continue;
		}

		iSet.insert(iLabel);
		jSet.insert(jLabel);
	}

	for (const auto iLabel : iSet)
	{
		for (const auto jLabel : jSet)
		{
			auto res = boost::add_edge(segmentToNode.at({i, iLabel}), segmentToNode.at({j, jLabel}), G);
			EdgeProperties& eProps = G[res.first];
			if (eProps.affinity == 0) { eProps.affinity = 1.0; }
			eProps.affinity += RIGID_MATCH_WEIGHT;
		}
	}
}

void reinforceObjectEdges(VideoSegmentationGraph<SEGMENT_TYPE::UCM>& G, const std::map<ros::Time, std::shared_ptr<SegmentationInfo>>& segmentations)
{
	using Vertex = VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor;
	using Edge = VideoSegmentationGraph<SEGMENT_TYPE::UCM>::edge_descriptor;

	for (Edge ed : make_range(boost::edges(G)))
	{
		Vertex u = boost::source(ed, G);
		Vertex v = boost::target(ed, G);

		EdgeProperties& ep = G[ed];
		const NodeProperties<SEGMENT_TYPE::UCM>& Gu = G[u];
		const NodeProperties<SEGMENT_TYPE::UCM>& Gv = G[v];

		if (Gu.t != Gv.t) { continue; }

		const std::shared_ptr<SegmentationInfo>& si = segmentations.at(Gu.t);

		const cv::Mat& C = si->centroids2;

		cv::Point2f cu(C.at<double>(Gu.leafID, 0)/2.0, C.at<double>(Gu.leafID, 1)/2.0);
		cv::Point2f cv(C.at<double>(Gv.leafID, 0)/2.0, C.at<double>(Gv.leafID, 1)/2.0);

		if (si->objectness_segmentation->image.at<uint16_t>(cu) == si->objectness_segmentation->image.at<uint16_t>(cv))
		{
			ep.affinity += SIFT_MATCH_WEIGHT / 5.0;
		}
	}
}

#include <pcl_ros/point_cloud.h>
void visualizeVideoGraph(const VideoSegmentationGraph<SEGMENT_TYPE::UCM>& G, const std::map<ros::Time, std::shared_ptr<SegmentationInfo>>& segmentations)//const std::vector<cv::Mat>& imgs, const std::vector<cv::Mat>& centroids)
{
	using Vertex = VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor;
	using Edge = VideoSegmentationGraph<SEGMENT_TYPE::UCM>::edge_descriptor;

	static ros::NodeHandle nh;
	static ros::Publisher pcPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("video_points", 1, true);
	static ros::Publisher edgePub = nh.advertise<visualization_msgs::Marker>("video_edges", 1, true);

	const float UV_SCALE = 0.001;
	const float T_SCALE = 0.3;
	const int IMAGE_SCALE = 2; // SEGMENT_TYPE::UCM is at double the resolution normal
	const ros::Time& startTime = segmentations.begin()->first;
	pcl::PointCloud<pcl::PointXYZRGB> pc;
//	size_t k = 0;
	for (const auto& pair : segmentations)
	{
		const cv::Mat& img = pair.second->display_contours;
		for (int u = 1; u < img.cols; ++u)
		{
			for (int v = 1; v < img.rows; ++v)
			{
				auto color = img.at<cv::Vec3b>(v, u);
				pcl::PointXYZRGB pt(color[2], color[1], color[0]);
				pt.x = UV_SCALE * (u + IMAGE_SCALE*pair.second->roi.x);
				pt.y = -UV_SCALE * (v + IMAGE_SCALE*pair.second->roi.y);
				pt.z = -T_SCALE * (pair.first-startTime).toSec();
				pc.points.push_back(pt);
			}
		}
//		++k;
	}

	pc.header.frame_id = "video";
	pcl_conversions::toPCL(ros::Time::now(), pc.header.stamp);
	pcPub.publish(pc);
	ros::spinOnce();

	visualization_msgs::Marker m;
	m.action = visualization_msgs::Marker::ADD;
	m.type = visualization_msgs::Marker::LINE_LIST;
	m.color.a = 1.0f;
	m.color.r = 1.0f;
	m.color.g = 1.0f;
	m.color.b = 1.0f;
	m.pose.orientation.w = 1;
	m.frame_locked = true;
	m.id = 1;
	m.scale.x = 1.1*UV_SCALE;

	double minAffinity = std::numeric_limits<double>::infinity();
	double maxAffinity = -std::numeric_limits<double>::infinity();

	for (Edge ed : make_range(boost::edges(G)))
	{
		const EdgeProperties& ep = G[ed];
		minAffinity = std::min(ep.affinity, minAffinity);
		maxAffinity = std::max(ep.affinity, maxAffinity);
	}

	geometry_msgs::Point pt;
	std_msgs::ColorRGBA color;
	for (Edge ed : make_range(boost::edges(G)))
	{
		Vertex u = boost::source(ed, G);
		Vertex v = boost::target(ed, G);

		const EdgeProperties& ep = G[ed];
		const NodeProperties<SEGMENT_TYPE::UCM>& Gu = G[u];
		const NodeProperties<SEGMENT_TYPE::UCM>& Gv = G[v];

		const std::shared_ptr<SegmentationInfo>& Su = segmentations.at(Gu.t);
		const std::shared_ptr<SegmentationInfo>& Sv = segmentations.at(Gv.t);

		const cv::Mat& Cu = Su->centroids2;
		const cv::Mat& Cv = Sv->centroids2;

		cv::Point2f cu(Cu.at<double>(Gu.leafID, 0), Cu.at<double>(Gu.leafID, 1));
		cv::Point2f cv(Cv.at<double>(Gv.leafID, 0), Cv.at<double>(Gv.leafID, 1));

		pt.x = UV_SCALE * (cu.x + IMAGE_SCALE*Su->roi.x);
		pt.y = -UV_SCALE * (cu.y + IMAGE_SCALE*Su->roi.y);
		pt.z = -T_SCALE * (Gu.t-startTime).toSec();
		m.points.push_back(pt);

//		colormap(igl::viridis_cm, (float)(ep.affinity-minAffinity)/(maxAffinity-minAffinity)), color.r, color.g, color.b);
		colormap(igl::viridis_cm, (float)tanh(10.0*(ep.affinity-minAffinity)/(maxAffinity-minAffinity)), color.r, color.g, color.b);
		m.colors.push_back(color);

		pt.x = UV_SCALE * (cv.x + IMAGE_SCALE*Sv->roi.x);
		pt.y = -UV_SCALE * (cv.y + IMAGE_SCALE*Sv->roi.y);
		pt.z = -T_SCALE * (Gv.t-startTime).toSec();
		m.points.push_back(pt);
		m.colors.push_back(color);
	}
	m.header.frame_id = pc.header.frame_id;
	m.header.stamp = ros::Time::now();

	edgePub.publish(m);
}

void visualizeObjectBundles(const VideoSegmentationGraph<SEGMENT_TYPE::BODY>& G, std::map<ros::Time, std::shared_ptr<SegmentationInfo>> segmentations, std::map<SegmentIndex<SEGMENT_TYPE::BODY>, BundleIndex> segmentToBundle, std::map<BundleIndex, cv::Scalar> bundleColors)//const std::vector<cv::Mat>& imgs, const std::vector<cv::Mat>& centroids)
{
	using Vertex = VideoSegmentationGraph<SEGMENT_TYPE::BODY>::vertex_descriptor;
	using Edge = VideoSegmentationGraph<SEGMENT_TYPE::BODY>::edge_descriptor;

	static ros::NodeHandle nh;
	static ros::Publisher pcPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("video_color", 1, true);
	static ros::Publisher edgePub = nh.advertise<visualization_msgs::Marker>("video_edge_colors", 1, true);

	const float UV_SCALE = 0.001;
	const float T_SCALE = 0.3;
	const ros::Time& startTime = segmentations.begin()->first;
	pcl::PointCloud<pcl::PointXYZRGB> pc;
	size_t k = 0;
	for (const auto& pair : segmentations)
	{
//		const cv::Mat& img = pair.second->labels;
		const cv::Mat& img = pair.second->objectness_segmentation->image;
		for (int u = 1; u < img.cols; ++u)
		{
			for (int v = 1; v < img.rows; ++v)
			{
				int label = img.at<uint16_t>(v, u);
				auto iter = segmentToBundle.find({pair.first, label});
				if (iter == segmentToBundle.end())
				{
//					std::cerr << "??" << std::endl;
					++k;
					continue;
				}
				BundleIndex bundle = iter->second;
				auto color = bundleColors.at(bundle);
				pcl::PointXYZRGB pt(color[2], color[1], color[0]);
				pt.x = UV_SCALE * (u+pair.second->roi.x);
				pt.y = -UV_SCALE * (v+pair.second->roi.y);
				pt.z = -T_SCALE * (pair.first-startTime).toSec();
				pc.points.push_back(pt);
			}
		}
//		++k;
	}
	std::cerr << "Unmatched pixels: " << k << std::endl;

	pc.header.frame_id = "video";
	pcl_conversions::toPCL(ros::Time::now(), pc.header.stamp);
	pcPub.publish(pc);
	ros::spinOnce();

	visualization_msgs::Marker m;
	m.action = visualization_msgs::Marker::ADD;
	m.type = visualization_msgs::Marker::LINE_LIST;
	m.color.a = 1.0f;
	m.color.r = 1.0f;
	m.color.g = 1.0f;
	m.color.b = 1.0f;
	m.pose.orientation.w = 1;
	m.frame_locked = true;
	m.id = 1;
	m.scale.x = 1.1*UV_SCALE;

	double minAffinity = std::numeric_limits<double>::infinity();
	double maxAffinity = -std::numeric_limits<double>::infinity();

	for (Edge ed : make_range(boost::edges(G)))
	{
		const EdgeProperties& ep = G[ed];
		minAffinity = std::min(ep.affinity, minAffinity);
		maxAffinity = std::max(ep.affinity, maxAffinity);
	}

	geometry_msgs::Point pt;
	std_msgs::ColorRGBA color;
	for (Edge ed : make_range(boost::edges(G)))
	{
		Vertex u = boost::source(ed, G);
		Vertex v = boost::target(ed, G);
		if (u == v) {throw std::logic_error("Graph has self-loops.");}

		const EdgeProperties& ep = G[ed];
		const NodeProperties<SEGMENT_TYPE::BODY>& Gu = G[u];
		const NodeProperties<SEGMENT_TYPE::BODY>& Gv = G[v];

		const std::shared_ptr<SegmentationInfo>& Su = segmentations.at(Gu.t);
		const std::shared_ptr<SegmentationInfo>& Sv = segmentations.at(Gv.t);

		cv::Moments mu = cv::moments(Su->objectness_segmentation->image == Gu.leafID);
		cv::Moments mv = cv::moments(Sv->objectness_segmentation->image == Gv.leafID);

		if (mu.m00 == 0.0) {throw std::runtime_error("");}
		if (mv.m00 == 0.0) {throw std::runtime_error("");}

		cv::Point2f cu(mu.m10/mu.m00, mu.m01/mu.m00);
		cv::Point2f cv(mv.m10/mv.m00, mv.m01/mv.m00);

		pt.x = UV_SCALE * (cu.x+Su->roi.x);
		pt.y = -UV_SCALE * (cu.y+Su->roi.y);
		pt.z = -T_SCALE * (Gu.t-startTime).toSec();
		m.points.push_back(pt);

		colormap(igl::viridis_cm, (float)((ep.affinity-minAffinity)/(maxAffinity-minAffinity)), color.r, color.g, color.b);
		m.colors.push_back(color);

		pt.x = UV_SCALE * (cv.x+Sv->roi.x);
		pt.y = -UV_SCALE * (cv.y+Sv->roi.y);
		pt.z = -T_SCALE * (Gv.t-startTime).toSec();
		m.points.push_back(pt);
		m.colors.push_back(color);
	}
	m.header.frame_id = pc.header.frame_id;
	m.header.stamp = ros::Time::now();

	edgePub.publish(m);

	ros::spinOnce();
	usleep(100000);
}

VideoSegmentationGraph<SEGMENT_TYPE::UCM> subgraph(const VideoSegmentationGraph<SEGMENT_TYPE::UCM>& G1, const std::vector<int>& labels, const int label)
{
	using Vertex = VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor;
	using Edge = VideoSegmentationGraph<SEGMENT_TYPE::UCM>::edge_descriptor;

	VideoSegmentationGraph<SEGMENT_TYPE::UCM> G2;

	SegmentLookup<SEGMENT_TYPE::UCM> segmentToNode;
	for (Vertex vd : make_range(boost::vertices(G1)))
	{
		if (labels[vd] == label)
		{
			const NodeProperties<SEGMENT_TYPE::UCM>& np = G1[vd];
			Vertex vd2 = boost::add_vertex(np, G2);
			auto r = segmentToNode.insert({{np.t, np.leafID}, vd2});
			MPS_ASSERT(r.second);
		}
	}


	for (const Edge ed : make_range(boost::edges(G1)))
	{
		Vertex u = boost::source(ed, G1);
		Vertex v = boost::target(ed, G1);

		const EdgeProperties& ep = G1[ed];
		const NodeProperties<SEGMENT_TYPE::UCM>& Gu = G1[u];
		const NodeProperties<SEGMENT_TYPE::UCM>& Gv = G1[v];

		VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor s, t;
		const auto a = segmentToNode.find({Gu.t, Gu.leafID});
		const auto b = segmentToNode.find({Gv.t, Gv.leafID});

		if (a == segmentToNode.end() || b == segmentToNode.end()) { continue; }

		s = a->second;
		t = b->second;

		boost::add_edge(s, t, ep, G2);
	}

	return G2;
}

void getUcmObjectMappings(const SegmentationInfo& si, std::map<uint16_t, uint16_t>& ucmLabelsToObjectLabel, std::map<uint16_t, std::set<uint16_t> >& objectLabelToUcmLabels)
{
	MPS_ASSERT(si.labels.size() == si.objectness_segmentation->image.size());
	const auto objectLabelIDs = unique(si.objectness_segmentation->image);

	for (const uint16_t objID : objectLabelIDs)
	{
		cv::Mat labelMask = (si.objectness_segmentation->image == objID);
		cv::Mat matches = cv::Mat::zeros(si.labels.size(), CV_16U);
		si.labels.copyTo(matches, labelMask);
		auto ucmLabelIDs = unique(matches);
		ucmLabelIDs.erase(0);

		objectLabelToUcmLabels[objID] = ucmLabelIDs;
		for (const uint16_t ucmID : ucmLabelIDs)
		{
			ucmLabelsToObjectLabel.insert({ucmID, objID});
		}
	}
}

VideoSegmentationGraph<SEGMENT_TYPE::BODY> compressGraph(const VideoSegmentationGraph<SEGMENT_TYPE::UCM>& G1, std::map<ros::Time, std::map<uint16_t, uint16_t>>& ucmLabelsToObjectLabel)
{

	VideoSegmentationGraph<SEGMENT_TYPE::BODY> G2;

	SegmentLookup<SEGMENT_TYPE::BODY>  segmentToNodeG2;

	for (const auto& p : ucmLabelsToObjectLabel)
	{
		const ros::Time& t = p.first;
		for (const auto& p2 : p.second)
		{
			if (0 == p2.first) { continue; }
			if (segmentToNodeG2.find({t, p2.second}) != segmentToNodeG2.end()) { continue; }
			VideoSegmentationGraph<SEGMENT_TYPE::BODY>::vertex_descriptor v = boost::add_vertex(NodeProperties<SEGMENT_TYPE::BODY>({t, p2.second}), G2);
			segmentToNodeG2.insert({{t, p2.second}, v});
		}
	}

	for (VideoSegmentationGraph<SEGMENT_TYPE::UCM>::edge_descriptor ed : make_range(boost::edges(G1)))
	{
		VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor u1 = boost::source(ed, G1);
		VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor v1 = boost::target(ed, G1);

		const EdgeProperties& ep = G1[ed];
		const NodeProperties<SEGMENT_TYPE::UCM>& Gu = G1[u1];
		const NodeProperties<SEGMENT_TYPE::UCM>& Gv = G1[v1];

		const ros::Time& i = Gu.t;
		const ros::Time& j = Gv.t;

		int iNewLabel = ucmLabelsToObjectLabel.at(i).at(Gu.leafID);
		int jNewLabel = ucmLabelsToObjectLabel.at(j).at(Gv.leafID);


		VideoSegmentationGraph<SEGMENT_TYPE::BODY>::vertex_descriptor u2 = segmentToNodeG2.at({i, iNewLabel});
		VideoSegmentationGraph<SEGMENT_TYPE::BODY>::vertex_descriptor v2 = segmentToNodeG2.at({j, jNewLabel});
		if (u2==v2)
		{
			continue;
		}

		auto res = boost::add_edge(u2, v2, G2);
		EdgeProperties& eProps = G2[res.first];
		eProps.affinity = std::max(eProps.affinity, ep.affinity);
	}

	typedef std::map<VideoSegmentationGraph<SEGMENT_TYPE::BODY>::vertex_descriptor, unsigned long> mapping_t;
	typedef boost::shared_ptr<mapping_t> vertex_component_map;

	vertex_component_map mapping = boost::make_shared<mapping_t>();
	size_t num_components = boost::connected_components(G2, boost::associative_property_map<mapping_t>(*mapping));
	if (num_components > 1)
	{
//		throw std::logic_error("Graph is disconnected.");
	}
	else if (num_components == 0)
	{
		throw std::logic_error("Graph is empty.");
	}
	else
	{
		std::cerr << "Compressed graph is connected." << std::endl;
	}

	return G2;
}

#include "mps_voxels/octree_utils.h"
#include <tf_conversions/tf_eigen.h>
//#include <rosgraph_msgs/Clock.h>
void visualize(const OctreeMotionModel& omm, tf::TransformBroadcaster* tb, const std::string& globalFrame, const std::string& frame)
{
	static ros::NodeHandle nh;
	static ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("object_memory", 1, true);
//	static ros::Publisher clockPub = nh.advertise<rosgraph_msgs::Clock>("/clock", 1, false);
//	static int clock_count = 0;

	ros::Time fakeNow = ros::Time::now();// + ros::Duration(++clock_count);
//	rosgraph_msgs::Clock clk;
//	clk.clock = fakeNow;
//	clockPub.publish(clk);

	tf::Transform T;
	tf::poseEigenToTF(omm.localTglobal.inverse(Eigen::Isometry), T);
	tb->sendTransform(tf::StampedTransform(T, fakeNow, globalFrame, frame));

	visualization_msgs::MarkerArray occupiedNodesVis = visualizeOctree(omm.octree.get(), frame);
	for (visualization_msgs::Marker& m : occupiedNodesVis.markers)
	{
		m.ns = frame;
		m.header.stamp = fakeNow;
	}
	pub.publish(occupiedNodesVis);
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "flow");
	ros::NodeHandle nh, pnh("~");

	setIfMissing(pnh, "frame_id", "table_surface");
	setIfMissing(pnh, "resolution", 0.010);

	auto listener = std::make_shared<tf::TransformListener>(ros::Duration(60.0));

	image_transport::ImageTransport it(nh);
//	image_transport::TransportHints hints("compressed", ros::TransportHints(), pnh);

	ros::Publisher pcPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("pile_points", 1, true);

	std::shared_ptr<robot_model_loader::RobotModelLoader> mpLoader
		= std::make_shared<robot_model_loader::RobotModelLoader>();
	robot_model::RobotModelPtr pModel = mpLoader->getModel();

	MPS_ASSERT(!pModel->getJointModelGroupNames().empty());

	std::map<std::string, std::shared_ptr<MotionModel>> selfModels;
	if (!loadLinkMotionModels(pModel.get(), selfModels))
	{
		ROS_ERROR("Model loading failed.");
	}
	selfModels.erase("victor_base_plate"); // HACK: camera always collides
	selfModels.erase("victor_pedestal");
	selfModels.erase("victor_left_arm_mount");
	selfModels.erase("victor_right_arm_mount");

	ros::ServiceClient segmentClient = nh.serviceClient<mps_msgs::SegmentGraph>("/segment_graph");
	if (!segmentClient.waitForExistence(ros::Duration(3)))
	{
		ROS_ERROR("Graph segmentation server not connected.");
		return -1;
	}

//	RigidClusterClient motionClient("/cluster_flow", true);
//	if (!motionClient.waitForServer(ros::Duration(3)))
//	{
//		ROS_ERROR("Flow segmentation server not connected.");
//	}
//
//	FrameMatchClient matchClient("/histogram_matcher", true);
//	if (!matchClient.waitForServer(ros::Duration(3)))
//	{
//		ROS_ERROR("Frame match server not connected.");
//	}


	std::shared_ptr<Scenario> scenario = std::make_shared<Scenario>();
	scenario->mapServer = std::make_shared<LocalOctreeServer>(pnh);
	scenario->listener = listener;
	scenario->broadcaster = std::make_shared<tf::TransformBroadcaster>();
	scenario->segmentationClient = std::make_shared<CachingRGBDSegmenter>(nh);
	scenario->robotModel = pModel;
//	scenario->loadManipulators(pModel);

//	std::unique_ptr<Scene> scene = std::make_unique<Scene>();
	std::map<ros::Time, std::shared_ptr<Scene>> scenes;

	segmentationPub = std::make_unique<image_transport::Publisher>(it.advertise("segmentation", 1));

	cv::namedWindow("segmentation", CV_WINDOW_NORMAL);
	cv::namedWindow("contours", CV_WINDOW_NORMAL);
	cv::namedWindow("labels", CV_WINDOW_NORMAL);

	const int step = 8;//3;
	tracker = std::make_unique<CudaTracker>(25*step, listener);
//	tracker = std::make_unique<Tracker>(5*step, listener);

	std::map<BundleIndex, std::shared_ptr<OctreeMotionModel>> omms;
//	OctreeMotionModel omm;
//	omm.localTglobal = MotionModel::Pose::Identity();
//	omm.octree = std::make_shared<octomap::OcTree>(scenario->mapServer->m_res);

	// TODO: Make step size variable based on average flow

	CachingRGBDSegmenter::SegmentationCache& segmentations = std::dynamic_pointer_cast<CachingRGBDSegmenter>(scenario->segmentationClient)->cache;

	while (ros::ok())
	{
		if (tracker->rgb_buffer.size() == tracker->MAX_BUFFER_LEN)
		{
			// Load tracking graph weights
			std::string param_ns = "/video_graph/";
			getSetIfMissing(nh, param_ns+"sift_weight", SIFT_MATCH_WEIGHT);
			getSetIfMissing(nh, param_ns+"rigid_weight", RIGID_MATCH_WEIGHT);
			pnh.param("num_labels", NUM_OUTPUT_LABELS, NUM_OUTPUT_LABELS);
			std::cerr << "Sift weight: " << SIFT_MATCH_WEIGHT << std::endl;

			cv::Size nativeSize = tracker->rgb_buffer.begin()->second->image.size();

			VideoSegmentationGraph<SEGMENT_TYPE::UCM> G;

			std::vector<ros::Time> steps;
			for (auto iter = tracker->rgb_buffer.begin(); iter != tracker->rgb_buffer.end(); std::advance(iter, step))
			{
				steps.push_back(iter->first);
			}

			tracker->track(steps);

			for (int i = 0; i < static_cast<int>(steps.size()) && ros::ok(); ++i)
			{
				const ros::Time& tCurr = steps[i];

				std::shared_ptr<Scene> scene = std::make_shared<Scene>();
				scenes.insert({tCurr, scene});

				scene->scenario = scenario;
				scene->selfModels = selfModels;

				scene->visualize = true;

				// NB: We do this every loop because we shrink the box during the crop/filter process
				scene->minExtent = Eigen::Vector4f(-0.4f, -0.6f, -0.020f, 1);
				scene->maxExtent = Eigen::Vector4f(0.4f, 0.6f, 0.5f, 1);
				scene->cv_rgb_ptr = tracker->rgb_buffer[tCurr];
				scene->cv_depth_ptr = tracker->depth_buffer[tCurr];
				sensor_msgs::CameraInfo cam_info = tracker->cameraModel.cameraInfo();
				cam_info.header.stamp = scene->cv_rgb_ptr->header.stamp;
				scene->cameraModel.fromCameraInfo(cam_info);

				bool sceneSuccess;
				sceneSuccess = scene->loadAndFilterScene();
				if (!sceneSuccess) { throw std::runtime_error(":("); }
				sceneSuccess = scene->performSegmentation();
				if (!sceneSuccess) { throw std::runtime_error(":("); }

				const SegmentationInfo& si = *segmentations.at(tCurr);

				double alpha = 0.75;
				cv::Mat labelColorsMap = colorByLabel(si.objectness_segmentation->image);
				labelColorsMap.setTo(0, 0 == si.objectness_segmentation->image);
				labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*si.rgb;
				cv::imshow("segmentation", labelColorsMap);
				cv::waitKey(1);

				cv::Mat displayLabels;
				si.labels.convertTo(displayLabels, CV_8UC1);
				cv::imshow("labels", displayLabels);

				cv::imshow("contours", si.display_contours);
				cv::waitKey(1);

				if (segmentationPub->getNumSubscribers() > 0)
				{
					segmentationPub->publish(cv_bridge::CvImage(tracker->cameraModel.cameraInfo().header, "bgr8", labelColorsMap).toImageMsg());
				}
				scene->pile_cloud->header.frame_id = scene->cameraFrame;
				pcl_conversions::toPCL(ros::Time::now(), scene->pile_cloud->header.stamp);
				pcPub.publish(scene->pile_cloud);
				ros::spinOnce();

				addToGraph(G, si);//.ucm2, si.labels2, tCurr);

				if (i >= 1)
				{
					const ros::Time& tPrev = steps[i-1];
//					const cv::Mat prevLabels = segmentations[tPrev]->labels;
					joinFrameGraphs(G, *segmentations[tPrev], si, tracker->flows2.at({tPrev, tCurr}));

					// Add rigid motion cliques
					/*
					if (motionClient.isServerConnected())
					{

						mps_msgs::ClusterRigidMotionsGoal req;
						req.flow_field.reserve(tracker->flows3.at({tPrev, tCurr}).size());
						std::vector<size_t> valid_flow_to_all_flow_lookup;
						for (size_t f = 0; f<tracker->flows3[{tPrev, tCurr}].size(); ++f)
						{
							const auto& flow = tracker->flows3[{tPrev, tCurr}][f];
							std::cerr << flow.second.norm() << std::endl;
							if (flow.second.norm()<0.01) { continue; }
							if (0.1>flow.first.z() || flow.first.z()>2.0) { continue; }
							mps_msgs::FlowVector flowVector;
							flowVector.pos.x = flow.first.x();
							flowVector.pos.y = flow.first.y();
							flowVector.pos.z = flow.first.z();
							flowVector.vel.x = flow.second.x();
							flowVector.vel.y = flow.second.y();
							flowVector.vel.z = flow.second.z();
							req.flow_field.push_back(flowVector);
							valid_flow_to_all_flow_lookup.push_back(f);
						}
						std::cerr << tracker->flows3[{tPrev, tCurr}].size() << "->" << req.flow_field.size() << std::endl;

						auto success = motionClient.sendGoalAndWait(req);
						if (!success.isDone()) { continue; }
						mps_msgs::ClusterRigidMotionsResultConstPtr resp = motionClient.getResult();
						if (resp)
						{
							MPS_ASSERT(resp->labels.size()==req.flow_field.size());
//						for (const auto l : resp->labels) { std::cerr << l << "\t"; } std::cerr << std::endl;

							std::map<int, std::vector<size_t>> flow_clusters;
							for (size_t f = 0; f<req.flow_field.size(); ++f)
							{
								flow_clusters[resp->labels[f]].push_back(f);
							}

							for (const auto& cluster : flow_clusters)
							{
								if (cluster.second.size()>10)
								{
									Tracker::Flow2D flows2;
									for (size_t iter = 0; iter<cluster.second.size(); ++iter)
									{
										// We skipped a bunch of flows, so we need to find our original position in the list
										size_t f = valid_flow_to_all_flow_lookup[iter];
										flows2.push_back(tracker->flows2[{tPrev, tCurr}][f]);
									}

//									joinMotionCliques(G, *segmentations[tPrev], si, flows2);
								}
							}
						}
					}
					*/
				}
				visualizeVideoGraph(G, segmentations);
			}

			std::map<ros::Time, std::map<uint16_t, uint16_t>> ucmLabelsToObjectLabelWithTime;
			for (int i = 0; i < static_cast<int>(steps.size()) && ros::ok(); ++i)
			{
				const ros::Time& t = steps[i];

				const SegmentationInfo& si = *segmentations[t];

				std::map<uint16_t, uint16_t> ucmLabelsToObjectLabel;
				std::map<uint16_t, std::set<uint16_t> > objectLabelToUcmLabels;
				getUcmObjectMappings(si, ucmLabelsToObjectLabel, objectLabelToUcmLabels);

				ucmLabelsToObjectLabelWithTime.insert({t, ucmLabelsToObjectLabel});
			}

			VideoSegmentationGraph<SEGMENT_TYPE::BODY> objectG = compressGraph(G, ucmLabelsToObjectLabelWithTime);

			// Filter for segments that have objects
//			{
//				boost::graph_traits<VideoSegmentationGraph<SEGMENT_TYPE::BODY>>::vertex_iterator vi, vi_end, next;
//				boost::tie(vi, vi_end) = boost::vertices(objectG);
//				for (next = vi; vi != vi_end; vi = next)
//				{
//					++next;
//					const auto seg = objectG[*vi];
//					const auto& lookup = scenes.at(seg.t)->labelToIndexLookup;
//					if (lookup.find(seg.leafID) == lookup.end())
//					{
//						std::cerr << "Rejected: <" << seg.t << "," << seg.leafID << ">." << std::endl;
//						boost::clear_vertex(*vi, objectG);
//						boost::remove_vertex(*vi, objectG);
//						// Remove invalidates iterators, start over
//						boost::tie(vi, vi_end) = boost::vertices(objectG);
//						next = vi;
//					}
//				}
//				MPS_ASSERT(boost::num_vertices(objectG) > 0);
//			}

			std::map<BundleIndex, cv::Scalar> bundle_colors;
			cv::RNG rng(0);

			MPS_ASSERT(segmentations.find(segmentations.rbegin()->first) != segmentations.end());
			MPS_ASSERT(segmentations.rbegin()->first == segmentations.rbegin()->second->t);
//			mps_msgs::MatchFrameSegmentsGoal g;
//			g.frames.emplace_back(toSegmentationMsg(*segmentations.begin()->second));
//			g.frames.emplace_back(toSegmentationMsg(*segmentations.rbegin()->second));
//			auto res = matchClient.sendGoalAndWait(g);
//			if (!res.isDone() || res.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) { throw std::runtime_error("Matching failed."); }
//
//			mps_msgs::MatchFrameSegmentsResultConstPtr frameMatchResult = matchClient.getResult();
//			if (!frameMatchResult) { continue; }
//			for (const auto& b : frameMatchResult->segments_to_bundles)
//			{
//				for (const auto v : b.values)
//				{
//					bundle_colors[BundleIndex{v}] = randomColor(rng);
//				}
//			}

//			std::map<SegmentIndex, int> segmentToBundle;

//			{
//				std::map<ros::Time, cv::Mat> video;
//				for (const ros::Time& t : frameMatchResult->stamps)
//				{
//					video.insert({t, cv::Mat::zeros(nativeSize, CV_8UC3)});
//				}
//
//				for (size_t i = 0; i < frameMatchResult->segments_to_bundles.size(); ++i)
//				{
//					const ros::Time& t = frameMatchResult->stamps[i];
//					const mps_msgs::IndexMap& b = frameMatchResult->segments_to_bundles[i];
//					for (size_t j = 0; j < b.keys.size(); ++j)
//					{
//						const auto& si = segmentations.at(t);
//						const int k = b.keys[j];
//						const BundleIndex v{b.values[j]};
////						segmentToBundle.insert({{t, k}, v});
//						cv::Mat active(video.at(t), si->roi);
//						MPS_ASSERT(active.size == segmentations.at(t)->labels.size);
//						active.setTo(bundle_colors.at(v), si->objectness_segmentation->image == k);
//
//						cv::imshow("source", si->rgb);
//						cv::imshow("mask", si->objectness_segmentation->image == k);
//						cv::imshow("video", video.at(t));
//	//								cv::waitKey(0);
//					}
//				}
//
//				for (const auto& pair : video)
//				{
//					cv::imshow("video", pair.second);
//					cv::waitKey(1);
//				}
//
//				cv::VideoWriter tracking("bundled_pair.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 1, nativeSize, true);
//				for (const auto& pair : video)
//					tracking.write(pair.second);
//				tracking.release();
//			}

			{/*
				// Maps from bundle ID (id) to all segments contained in that "object". Reverse lookup of segments_to_bundles
				std::map<BundleIndex, std::set<SegmentIndex<SEGMENT_TYPE::BODY>> > bundleMembership;
				for (size_t t = 0; t < frameMatchResult->stamps.size(); ++t)
				{
					const auto& lookup = scenes.at(frameMatchResult->stamps[t])->labelToIndexLookup;
					MPS_ASSERT(!lookup.empty());
					for (size_t i = 0; i < frameMatchResult->segments_to_bundles[t].keys.size(); ++i)
					{
						// Not all segments wind up in our final structure, due to being background, or too small, etc
						if (lookup.find(frameMatchResult->segments_to_bundles[t].keys[i]) == lookup.end())
						{
							std::cerr << "SegmentID <" << frameMatchResult->stamps[t] << "," << frameMatchResult->segments_to_bundles[t].keys[i] << "> was rejected." << std::endl;
							continue;
						}
						else
						{
							std::cerr << "SegmentID <" << frameMatchResult->stamps[t] << "," << frameMatchResult->segments_to_bundles[t].keys[i] << "> was located." << std::endl;
						}

						bundleMembership[BundleIndex{frameMatchResult->segments_to_bundles[t].values[i]}].insert({frameMatchResult->stamps[t], frameMatchResult->segments_to_bundles[t].keys[i]}); // TODO: Problem here???
					}
				}

				for (auto& bundle : bundleMembership)
				{
					std::shared_ptr<OctreeMotionModel> omm = std::make_shared<OctreeMotionModel>();
					omm->localTglobal = MotionModel::Pose::Identity();
					omm->octree = std::make_shared<octomap::OcTree>(scenario->mapServer->m_res);
					omms.insert({bundle.first, omm});
				}

				// Map from a segment ID to a vertex ID in the graph
				SegmentLookup<SEGMENT_TYPE::BODY> objectSegmentToNode;
				for (VideoSegmentationGraph<SEGMENT_TYPE::BODY>::vertex_descriptor vd : make_range(boost::vertices(objectG)))//objectG
				{
					const NodeProperties<SEGMENT_TYPE::BODY>& np = objectG[vd];
					auto r = objectSegmentToNode.insert({{np.t, np.leafID}, vd}); MPS_ASSERT(r.second);
				}

				std::map<SegmentIndex<SEGMENT_TYPE::BODY>, BundleIndex> segmentToBundle;
				visualizeObjectBundles(objectG, segmentations, segmentToBundle, bundle_colors);

				// Eventually the matcher should handle this part...
				// Connect from start frame to final frame
				for (auto& bundle : bundleMembership)
				{
					std::deque<SegmentIndex<SEGMENT_TYPE::BODY>> path = getObjectPath(objectG, objectSegmentToNode, *bundle.second.begin(), *bundle.second.rbegin());
					for (const auto& seg : path)
					{
						bundle.second.insert(seg);

						segmentToBundle[seg] = bundle.first;
					}

					visualizeObjectBundles(objectG, segmentations, segmentToBundle, bundle_colors);
				}

				for (const auto& bundle : bundleMembership)
				{
					std::map<ros::Time, cv::Mat> bundleMasks; ///< Collection of frames where this bundle was observed
					for (const auto& seg : bundle.second)
					{
						const auto& si = segmentations.at(seg.first);
						bundleMasks.insert({seg.first, cv::Mat::zeros(si->objectness_segmentation->image.size(), CV_8UC1)});
					}

					for (const auto& seg : bundle.second)
					{
						const auto& si = segmentations.at(seg.first);
						bundleMasks.at(seg.first).setTo(255, si->objectness_segmentation->image == seg.second);
					}

					// Step through pairs of masks and look for tracks
					auto iter1 = bundleMasks.begin();
					auto iter2 = ++bundleMasks.begin();

					for (; iter2 != bundleMasks.end(); ++iter1, ++iter2)
					{
						const auto& si1 = segmentations.at(iter1->first);
						const auto& si2 = segmentations.at(iter2->first);

						const auto& flow2 = tracker->flows2[{si1->t, si2->t}];
						const auto& flow3 = tracker->flows3[{si1->t, si2->t}];
						Tracker::Flow3D bundleFlow3;
						for (size_t f = 0; f < flow2.size(); ++f)
						{
							auto flowVec = flow2[f];

							if (!si1->roi.contains(cv::Point2i(flowVec.first.x, flowVec.first.y))) { continue; }
							if (!si2->roi.contains(cv::Point2i(flowVec.second.x, flowVec.second.y))) { continue; }

							flowVec.first.x -= si1->roi.x;
							flowVec.first.y -= si1->roi.y;
							flowVec.second.x -= si2->roi.x;
							flowVec.second.y -= si2->roi.y;

							cv::Point2i iPt(flowVec.first.x, flowVec.first.y);
							cv::Point2i jPt(flowVec.second.x, flowVec.second.y);

							// If flow2d @ time t1 in mask1 and flow2d in mask2 @ time t2, add to list
							if (iter1->second.at<uint8_t>(iPt) > 0
							    && iter2->second.at<uint8_t>(jPt) > 0)
							{
								bundleFlow3.push_back(flow3[f]);
							}
						}

						if (bundleFlow3.size() < 3)
						{
							std::cerr << "Insufficient matches (" << bundleFlow3.size() << ") for bundle '"
							          << bundle.first.id << "' between " << si1->t << " and " << si2->t << std::endl;
							continue;
						}

						Eigen::Isometry3d bTa;
						if (estimateRigidTransform(bundleFlow3, bTa))
						{
							std::cerr << "Got one: " << std::endl;
							std::cerr << bTa.matrix() << std::endl;

							const auto& scene1 = scenes.at(si1->t);
							const auto& scene2 = scenes.at(si2->t);

							for (const auto& seg : bundle.second)
							{
								if (seg.first == si1->t)
								{
									// NB: objectID may be missing if it took a stupid path through the inner frames
									const auto& iter = scene1->labelToIndexLookup.find(seg.second);
									if (iter == scene1->labelToIndexLookup.end()) { continue; }

									ObjectIndex obj = iter->second;
									std::shared_ptr<OctreeMotionModel>& omm = omms.at(bundle.first);

//									omm.localTglobal *= bTa;
									omm->observe(scene1->segments.at(obj), scene1->worldTcamera);

									visualize(*omm, scenario->broadcaster.get(), scene1->worldFrame, "1");
//									sleep(2);
								}
							}
//							omm.localTglobal.translation().x() += 0.1;
//							visualize(omm, scenario->broadcaster.get(), scene1->worldFrame, "1");

							// TODO: Draw the frame-frame connection here!


						}
						else
						{
							std::cerr << "RANSAC failed for bundle '"
							          << bundle.first.id << "' between " << si1->t << " and " << si2->t << std::endl;
						}

					}
				}

				for (const auto& bundle : bundleMembership)
				{
					for (const auto& seg : bundle.second)
					{
						segmentToBundle[seg] = bundle.first;
					}
				}
				visualizeObjectBundles(objectG, segmentations, segmentToBundle, bundle_colors);


				for (int i = 0; i < static_cast<int>(steps.size()) && ros::ok(); ++i)
				{
//					const ros::Time& tCurr = steps[i];
//					const ros::Time& tPrev = steps[i-1];
//
//					const std::shared_ptr<Scene>& sceneCurr = scenes.at(tCurr);
//					const std::shared_ptr<Scene>& scenePrev = scenes.at(tPrev);



//					omm.observe(scene->segments.front(), scene->worldTcamera);
//					visualize(omm, scenario->broadcaster.get(), scene->worldFrame, "1");
//					sleep(2);
//					omm.localTglobal.translation().x() += 0.1;
//					visualize(omm, scenario->broadcaster.get(), scene->worldFrame, "1");
				}
			*/}

//			std::vector<ComponentGraph> component_graphs;
//
//			for (size_t i = 0; i < num_components; i++)
//				component_graphs.emplace_back(G,
//				                              [mapping,i,&G](VideoSegmentationGraph::edge_descriptor e) {
//					                              return mapping->at(boost::source(e,G))==i
//					                                     || mapping->at(boost::target(e,G))==i;
//				                              },
//				                              [mapping,i](VideoSegmentationGraph::vertex_descriptor v) {
//					                              return mapping->at(v)==i;
//				                              });
//
//			for (const auto& g : component_graphs)
			{
//				Eigen::MatrixXd laplacian = getLaplacianNormalized(G);
//				const int numCells = laplacian.rows();
//				std::cerr << "Segmenting subgraph with " << numCells << " vertices (out of " << boost::num_vertices(G) << ")." << std::endl;
//				Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver;
//				solver.compute(laplacian, Eigen::ComputeEigenvectors);
//				Eigen::VectorXd eigenvalues = solver.eigenvalues();
//				Eigen::MatrixXd eigenvectors = solver.eigenvectors();
			}
//			for (size_t node = 0; node < componentLookup.size(); ++node)
//			{
//				int seg = componentLookup[node];
//
//			}

			{
				reinforceObjectEdges(G, segmentations);
				visualizeVideoGraph(G, segmentations);
			}

			int maxObjs = std::numeric_limits<int>::lowest();
			for (const auto& scene : scenes)
			{
				maxObjs = std::max(maxObjs, static_cast<int>(scene.second->segments.size()));
			}

			std::vector<int> componentLookup(boost::num_vertices(G));
			int num_components = boost::connected_components(G, &componentLookup[0]);
			std::vector<VideoSegmentationGraph<SEGMENT_TYPE::UCM>> Gs;
			for (int component = 0; component < num_components; ++component)
			{
				const auto Gprime = subgraph(G, componentLookup, component);
				if (static_cast<int>(boost::num_vertices(Gprime)) > maxObjs * 2)
				{
					Gs.push_back(Gprime);
				}
			}

			SegmentLookup<SEGMENT_TYPE::UCM> segmentToNode;
			for (VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor vd : make_range(boost::vertices(G)))
			{
				const NodeProperties<SEGMENT_TYPE::UCM>& np = G[vd];
				segmentToNode[{np.t, np.leafID}] = vd;
			}

			for (int numObjs = maxObjs - 2; numObjs < maxObjs + 4; ++numObjs)
			{
				bundle_colors.clear();
				std::map<VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor, int> objectClusters;
				int total_num_labels = 0;
				for (const auto& Gprime : Gs)
				{
					Eigen::SparseMatrix<double> adj = getAdjacencySparse(Gprime);
					mps_msgs::SegmentGraphRequest req;
					for (const auto& triplet : to_triplets(adj))
					{
						req.adjacency.row_index.push_back(triplet.row());
						req.adjacency.col_index.push_back(triplet.col());
						req.adjacency.value.push_back(triplet.value());
					}
					req.num_labels = numObjs;
					req.algorithm = "spectral"; //"affinity"; //dbscan

					mps_msgs::SegmentGraphResponse resp;
					bool success = segmentClient.call(req, resp);
					if (!success)
					{
						ROS_ERROR_STREAM("Graph segmentation call failed.");
						break;
					}

					for (VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor vd : make_range(boost::vertices(Gprime)))
					{
						const NodeProperties<SEGMENT_TYPE::UCM>& np = Gprime[vd];
						objectClusters.insert({segmentToNode.at({np.t, np.leafID}), resp.labels[vd]+total_num_labels});
					}
					total_num_labels += resp.num_labels;
				}

				for (const auto& oc : objectClusters)
				{
					bundle_colors[BundleIndex{oc.second}] = randomColor(rng);
				}

				// Recolor image based on segments
				std::map<ros::Time, cv::Mat> video;
				for (const ros::Time& t : steps)
				{
					video.insert({t, cv::Mat::zeros(nativeSize, CV_8UC3)});
				}

				for (VideoSegmentationGraph<SEGMENT_TYPE::UCM>::vertex_descriptor vd : make_range(boost::vertices(G)))
				{
					const NodeProperties<SEGMENT_TYPE::UCM>& np = G[vd];
					const auto& segmentation = segmentations.at(np.t);
					cv::Mat ucmLabelMask = segmentation->labels==np.leafID;
//					ucmLabelMask.setTo(0, segmentation->objectness_segmentation->image == 0);
					const auto iter = objectClusters.find(vd);
					if (iter != objectClusters.end())
					{
						cv::Mat(video.at(np.t), segmentation->roi).setTo(bundle_colors.at(BundleIndex{iter->second}), ucmLabelMask);
					}
				}

				const double alpha = 0.75;
				cv::VideoWriter tracking("clustered_"+std::to_string(numObjs)+".avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), 1, nativeSize, true);
				for (const auto& pair : video)
				{
					tracking.write(alpha*pair.second+(1.0-alpha)*scenes.at(pair.first)->cv_rgb_ptr->image);
				}
				tracking.release();
			}


//			{
//				Eigen::SparseMatrix<double> laplacian = getLaplacianSparseNormalized(G);
//				using SparseMatProd = Spectra::SparseSymMatProd<double>; // Spectra::SparseGenMatProd<double>
//				SparseMatProd op(laplacian);
//				Spectra::SymEigsSolver<double, Spectra::LARGEST_MAGN, SparseMatProd> eigs(&op, 5, 5*30+1);
//				eigs.init();
//				int nconv = eigs.compute();
//				if (eigs.info()==Spectra::SUCCESSFUL)
//				{
//					std::cerr << nconv << "\n" << eigs.eigenvectors().real() << std::endl;
//				}
//				else
//				{
//					ROS_WARN("Failed to compute eigenvectors. Clusterign aborted.");
//
//					tracker->reset();
//					continue;
//				}
//			}


			while (ros::ok() && cv::waitKeyEx(10) < 0)
			{
			}
			tracker->reset();
			std::cerr << "Tracker reset." << std::endl;
			tracker->startCapture();
		}
		cv::waitKey(1);
		sleep(1);
		cv::waitKey(1);
	}

	tracker.reset();
	segmentationClient.reset();
	segmentationPub.reset();
	return 0;
}