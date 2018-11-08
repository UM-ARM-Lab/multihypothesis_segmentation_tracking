//
// Created by arprice on 8/13/18.
//

//#include <opencv2/imgproc.hpp>
//#include <opencv2/optflow.hpp>
#include <opencv2/highgui.hpp>
//#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
//
//#include <cudaSift/image.h>
//#include <cudaSift/sift.h>
//#include <cudaSift/utils.h>
//
//#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <image_transport/subscriber_filter.h>
//#include <depth_image_proc/depth_conversions.h>
//#include <message_filters/subscriber.h>
//#include <message_filters/time_synchronizer.h>

#include "mps_voxels/CudaTracker.h"
#include "mps_voxels/image_utils.h"
#include "mps_voxels/segmentation_utils.h"
#include "mps_voxels/map_graph.h"
#include "mps_voxels/Ultrametric.h"
#include "mps_voxels/graph_matrix_utils.h"

#include <mps_msgs/SegmentGraph.h>
#include <mps_msgs/ClusterRigidMotions.h>

#include <visualization_msgs/MarkerArray.h>
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>

#include <boost/graph/filtered_graph.hpp>
#include <boost/graph/connected_components.hpp>

#include <Eigen/Eigenvalues>
#include <Eigen/SparseCore>
//#include <Spectra/GenEigsSolver.h>
//#include <Spectra/MatOp/SparseGenMatProd.h>
//#include <Spectra/SymEigsSolver.h>
//#include <Spectra/MatOp/SparseSymMatProd.h>

#include <ros/ros.h>

std::unique_ptr<Tracker> tracker;
std::shared_ptr<RGBDSegmenter> segmentationClient;
std::unique_ptr<image_transport::Publisher> segmentationPub;

double SIFT_MATCH_WEIGHT = 1.0/5.0;
int NUM_OUTPUT_LABELS = 15;
int pyrScale = 2;

template <typename T>
void setIfMissing(ros::NodeHandle& nh, const std::string& param_name, const T& param_val)
{
	if (!nh.hasParam(param_name))
	{
		nh.setParam(param_name, param_val);
	}
}

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

void cutGraph(VideoSegmentationGraph& graph, const std::vector<size_t>& cuts, const Eigen::MatrixXd& eigenvectors)
{
	std::set<VideoSegmentationGraph::edge_descriptor> cutEdges;
	for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::edges(graph)))
	{
		VideoSegmentationGraph::vertex_descriptor u = boost::source(ed, graph);
		VideoSegmentationGraph::vertex_descriptor v = boost::target(ed, graph);

		for (const size_t c : cuts)
		{
			if (sgn(eigenvectors.col(c)[u]) != sgn(eigenvectors.col(c)[v]))
			{
				cutEdges.insert(ed);
				break;
			}
		}
	}

	for (VideoSegmentationGraph::edge_descriptor ed : cutEdges)
	{
		boost::remove_edge(ed, graph);
	}
}

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


void addToGraph(VideoSegmentationGraph& G, cv::Mat& ucm, cv::Mat& full_labels, const int k)
{
	const double EDGE_THRESHOLD = 0.005;//0.125;

//	cv::Mat display_labels = (small_labels == 0);
//	display_labels.convertTo(display_labels, CV_8UC1);
//	cv::imshow("small_labels", display_labels);
//	cv::waitKey();

	Ultrametric um(ucm, full_labels);

	// Frame index, label value
	std::map<std::pair<int, int>, VideoSegmentationGraph::vertex_descriptor> segmentToNode;

	for (const auto lbl : unique(full_labels))
	{
		if (0 == lbl) { continue; }
		VideoSegmentationGraph::vertex_descriptor v = boost::add_vertex(NodeProperties({k, lbl}), G);
		segmentToNode.insert({{k, lbl}, v});
	}

//	std::map<label_type, std::vector<label_type>> preclusters = um.getCutClusters(EDGE_THRESHOLD);
//
//	for (const auto& precluster : preclusters)
//	{
//		for (int i = 0; i<static_cast<int>(precluster.second.size()); ++i)
//		{
//			for (int j = i+1; j<static_cast<int>(precluster.second.size()); ++j)
//			{
//				label_type p = precluster.second[i];
//				label_type q = precluster.second[j];
//				double dist = um.distance(p, q, EDGE_THRESHOLD);
//				if (dist < 0) { continue; }
//
//				assert(segmentToNode.find({k, p}) != segmentToNode.end());
//				assert(segmentToNode.find({k, q}) != segmentToNode.end());
//
//				VideoSegmentationGraph::vertex_descriptor s = segmentToNode[{k, p}];
//				VideoSegmentationGraph::vertex_descriptor t = segmentToNode[{k, q}];
//				boost::add_edge(s, t, {1.0/(dist+1e-4)}, G);
//			}
//		}
//	}

//	auto neighbor_pairs = computeNeighbors(full_labels);
//	for (const std::pair<uint16_t, uint16_t> n : neighbor_pairs)
//	{
//		label_type p = n.first;
//		label_type q = n.second;
//		double dist = um.distance(p, q, 1.0);
//		if (dist < 0) { throw std::runtime_error("Ultrametric max distance should be <= 1.0"); }
//
//		assert(segmentToNode.find({k, p}) != segmentToNode.end());
//		assert(segmentToNode.find({k, q}) != segmentToNode.end());
//
//		VideoSegmentationGraph::vertex_descriptor s = segmentToNode[{k, p}];
//		VideoSegmentationGraph::vertex_descriptor t = segmentToNode[{k, q}];
//		boost::add_edge(s, t, {1.0/(dist+1e-4)}, G);
//	}

	auto neighbor_pairs = computeUcmDual(full_labels, ucm);
	for (const auto edge : neighbor_pairs)
	{
		label_type p = edge.first.first;
		label_type q = edge.first.second;
		double dist = edge.second;
		if (dist <= 0 || 1.0 < dist) { throw std::runtime_error("Ultrametric max distance should be <= 1.0"); }

		assert(segmentToNode.find({k, p}) != segmentToNode.end());
		assert(segmentToNode.find({k, q}) != segmentToNode.end());

		VideoSegmentationGraph::vertex_descriptor s = segmentToNode[{k, p}];
		VideoSegmentationGraph::vertex_descriptor t = segmentToNode[{k, q}];
		boost::add_edge(s, t, {1.0/(dist+1e-4)}, G);
	}

	for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(G)))
	{
		assert(boost::out_degree(vd, G) > 0);
	}
}

void joinFrameGraphs(VideoSegmentationGraph& G, const cv::Mat& iLabels, const cv::Mat& jLabels, const int i, const int j, const Tracker::Flow2D& flow)
{
	std::map<std::pair<int, int>, VideoSegmentationGraph::vertex_descriptor> segmentToNode;
	for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(G)))
	{
		const NodeProperties& np = G[vd];
		segmentToNode[{np.k, np.leafID}] = vd;
	}
	for (const std::pair<cv::Point2f, cv::Point2f>& flowVec : flow)
	{
		cv::Point2i iPt(flowVec.first.x/pyrScale, flowVec.first.y/pyrScale);
		if (iPt.x >= iLabels.cols || iPt.y >= iLabels.rows) { continue; }
		cv::Point2i jPt(flowVec.second.x/pyrScale, flowVec.second.y/pyrScale);
		if (jPt.x >= jLabels.cols || jPt.y >= jLabels.rows) { continue; }
		int iLabel = iLabels.at<uint16_t>(iPt);
		int jLabel = jLabels.at<uint16_t>(jPt);

		if (iLabel == 0 || jLabel == 0)
		{
			continue;
		}

		auto res = boost::add_edge(segmentToNode.at({i, iLabel}), segmentToNode.at({j, jLabel}), G);
		EdgeProperties& eProps = G[res.first];
		if (eProps.affinity == 0) { eProps.affinity = 1.0; }
		eProps.affinity += SIFT_MATCH_WEIGHT;
	}
}

void joinMotionCliques(VideoSegmentationGraph& G, const cv::Mat& iLabels, const cv::Mat& jLabels, const int i, const int j, const Tracker::Flow2D& flow)
{
	std::map<std::pair<int, int>, VideoSegmentationGraph::vertex_descriptor> segmentToNode;
	for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(G)))
	{
		const NodeProperties& np = G[vd];
		segmentToNode[{np.k, np.leafID}] = vd;
	}

	std::set<uint16_t> iSet, jSet;

	for (const std::pair<cv::Point2f, cv::Point2f>& flowVec : flow)
	{
		cv::Point2i iPt(flowVec.first.x/pyrScale, flowVec.first.y/pyrScale);
		if (iPt.x >= iLabels.cols || iPt.y >= iLabels.rows) { continue; }
		cv::Point2i jPt(flowVec.second.x/pyrScale, flowVec.second.y/pyrScale);
		if (jPt.x >= jLabels.cols || jPt.y >= jLabels.rows) { continue; }
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
			eProps.affinity += SIFT_MATCH_WEIGHT;
		}
	}
}

#include <pcl_ros/point_cloud.h>
void visualizeVideoGraph(const VideoSegmentationGraph& G, const std::vector<cv::Mat>& imgs, const std::vector<cv::Mat>& centroids)
{
	static ros::NodeHandle nh;
	static ros::Publisher pcPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("video_points", 1, true);
	static ros::Publisher edgePub = nh.advertise<visualization_msgs::Marker>("video_edges", 1, true);

	const float UV_SCALE = 0.001;
	const float T_SCALE = 0.3;
	pcl::PointCloud<pcl::PointXYZRGB> pc;
	for (size_t k = 0; k < imgs.size(); ++k)
	{
		const cv::Mat& img = imgs[k];
		for (int u = 1; u < img.cols; ++u)
		{
			for (int v = 1; v < img.rows; ++v)
			{
				auto color = img.at<cv::Vec3b>(v, u);
				pcl::PointXYZRGB pt(color[2], color[1], color[0]);
				pt.x = UV_SCALE * u;
				pt.y = -UV_SCALE * v;
				pt.z = -T_SCALE * k;
				pc.points.push_back(pt);
			}
		}
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

	for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::edges(G)))
	{
		const EdgeProperties& ep = G[ed];
		minAffinity = std::min(ep.affinity, minAffinity);
		maxAffinity = std::max(ep.affinity, maxAffinity);
	}

	geometry_msgs::Point pt;
	std_msgs::ColorRGBA color;
	for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::edges(G)))
	{
		VideoSegmentationGraph::vertex_descriptor u = boost::source(ed, G);
		VideoSegmentationGraph::vertex_descriptor v = boost::target(ed, G);

		const EdgeProperties& ep = G[ed];
		const NodeProperties& Gu = G[u];
		const NodeProperties& Gv = G[v];

		const cv::Mat& Cu = centroids[Gu.k];
		const cv::Mat& Cv = centroids[Gv.k];

		cv::Point2f cu(Cu.at<double>(Gu.leafID, 0), Cu.at<double>(Gu.leafID, 1));
		cv::Point2f cv(Cv.at<double>(Gv.leafID, 0), Cv.at<double>(Gv.leafID, 1));

		pt.x = UV_SCALE * cu.x;
		pt.y = -UV_SCALE * cu.y;
		pt.z = -T_SCALE * Gu.k;
		m.points.push_back(pt);

		colormap(igl::viridis_cm, (float)((ep.affinity-minAffinity)/(maxAffinity-minAffinity)), color.r, color.g, color.b);
		m.colors.push_back(color);

		pt.x = UV_SCALE * cv.x;
		pt.y = -UV_SCALE * cv.y;
		pt.z = -T_SCALE * Gv.k;
		m.points.push_back(pt);
		m.colors.push_back(color);
	}
	m.header.frame_id = pc.header.frame_id;
	m.header.stamp = ros::Time::now();

	edgePub.publish(m);
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "flow");
	ros::NodeHandle nh, pnh("~");

	ros::ServiceClient segmentClient = nh.serviceClient<mps_msgs::SegmentGraph>("/segment_graph");
	if (!segmentClient.waitForExistence(ros::Duration(3)))
	{
		ROS_ERROR("Graph segmentation server not connected.");
		return -1;
	}

	ros::ServiceClient motionClient = nh.serviceClient<mps_msgs::ClusterRigidMotions>("/cluster_flow");
	if (!motionClient.waitForExistence(ros::Duration(3)))
	{
		ROS_ERROR("Flow segmentation server not connected.");
	}

	pnh.param("num_labels", NUM_OUTPUT_LABELS, NUM_OUTPUT_LABELS);
	pnh.param("sift_weight", SIFT_MATCH_WEIGHT, SIFT_MATCH_WEIGHT);
	std::cerr << "Sift weight: " << SIFT_MATCH_WEIGHT << std::endl;

//	image_transport::ImageTransport it(nh);
//	image_transport::TransportHints hints("compressed", ros::TransportHints(), pnh);
	image_transport::ImageTransport it(nh);

	segmentationClient = std::make_shared<RGBDSegmenter>(nh);
	segmentationPub = std::make_unique<image_transport::Publisher>(it.advertise("segmentation", 1));

	cv::namedWindow("segmentation", CV_WINDOW_NORMAL);
	cv::namedWindow("contours", CV_WINDOW_NORMAL);
	cv::namedWindow("labels", CV_WINDOW_NORMAL);

	const int step = 5;
	tracker = std::make_unique<CudaTracker>(3*step);

	// TODO: Make step size variable based on average flow

	std::vector<cv::Mat> ucms;
	std::vector<cv::Mat> labels;
	std::vector<cv::Mat> centroids;
	std::vector<cv::Mat> displays;
	std::vector<cv_bridge::CvImagePtr> segmentations;
	VideoSegmentationGraph G;

	while (ros::ok())
	{
		if (tracker->rgb_buffer.size() == tracker->MAX_BUFFER_LEN)
		{
			tracker->track(step);
			for (int k = 0; k < static_cast<int>(tracker->MAX_BUFFER_LEN)/step; ++k)
			{
				int i = k*step; ///< Index into image buffer

				cv::Rect roi(500, 500,
				             1000, 500);

				cv::Mat rgb_cropped(tracker->rgb_buffer[i]->image, roi); cv::pyrDown(rgb_cropped, rgb_cropped);// cv::pyrDown(rgb_cropped, rgb_cropped);
				cv::Mat depth_cropped(tracker->depth_buffer[i]->image, roi); cv::pyrDown(depth_cropped, depth_cropped);// cv::pyrDown(depth_cropped, depth_cropped);
				cv_bridge::CvImage cv_rgb_cropped(tracker->rgb_buffer[i]->header, tracker->rgb_buffer[i]->encoding, rgb_cropped);
				cv_bridge::CvImage cv_depth_cropped(tracker->depth_buffer[i]->header, tracker->depth_buffer[i]->encoding, depth_cropped);
				sensor_msgs::CameraInfo cam_msg_cropped = tracker->cameraModel.cameraInfo();
				cam_msg_cropped.width = roi.width/pyrScale;
				cam_msg_cropped.height = roi.height/pyrScale;
				cam_msg_cropped.K[2] -= roi.x/pyrScale;
				cam_msg_cropped.K[5] -= roi.y/pyrScale;
				for (auto& d : cam_msg_cropped.D) { d = 0.0; }

				cv_bridge::CvImagePtr ucm;
				segmentations.emplace_back(segmentationClient->segment(cv_rgb_cropped, cv_depth_cropped, cam_msg_cropped, &ucm));
				if (!segmentations.back())
				{
					ROS_ERROR_STREAM("Segmentation failed.");
					break;
				}
				ucms.push_back(ucm->image);
				cv::Mat tempLabels, tempCentroids, stats;
				cv::Mat silly_ucm; ucm->image.copyTo(silly_ucm);
//				cv::Mat silly_labels_small(silly_ucm.rows/2, silly_ucm.cols/2, CV_16UC1);
				// Not sure if necessary...
//				for (int u = 0; u < silly_ucm.cols; u+=2)
//				{
//					for (int v = 0; v < silly_ucm.rows; v+=2)
//					{
//						silly_ucm.at<double>(v, u) = 1.0;
//					}
//				}
				cv::connectedComponentsWithStats(silly_ucm == 0, tempLabels, stats, tempCentroids, 8, CV_16U);
//				for (int u = 1; u < tempLabels.cols; u+=2)
//				{
//					for (int v = 1; v < tempLabels.rows; v+=2)
//					{
//						silly_labels_small.at<uint16_t>(v/2, u/2) = tempLabels.at<uint16_t>(v, u);
//					}
//				}

				labels.push_back(tempLabels);
				centroids.push_back(tempCentroids);

				double alpha = 0.75;
				cv::Mat labelColorsMap = colorByLabel(segmentations.back()->image);
				labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*rgb_cropped;
				cv::imshow("segmentation", labelColorsMap);
				cv::waitKey(1);

				cv::Mat displayLabels;
				tempLabels.convertTo(displayLabels, CV_8UC1);
				cv::imshow("labels", displayLabels);

				cv::Mat tempContours1, displayContours;
				double maxVal;
				cv::minMaxLoc(ucm->image, nullptr, &maxVal);
				ucm->image.convertTo(tempContours1, CV_8UC1, 255.0/maxVal);
				cv::applyColorMap(tempContours1, displayContours, cv::COLORMAP_BONE);//cv::COLORMAP_PARULA);//cv::COLORMAP_JET); // COLORMAP_HOT
				cv::imshow("contours", displayContours);
				displays.push_back(displayContours);
//				displays.push_back(rgb_cropped);
				cv::waitKey(1);

				if (segmentationPub->getNumSubscribers() > 0)
				{
					segmentationPub->publish(cv_bridge::CvImage(tracker->cameraModel.cameraInfo().header, "bgr8", labelColorsMap).toImageMsg());
				}

				addToGraph(G, ucms.back(), tempLabels, i/step);

//				if (i == 0)
//				{
//					int edgeCount = 0;
//					for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::edges(G)))
//					{
//						VideoSegmentationGraph::vertex_descriptor u = boost::source(ed, G);
//						VideoSegmentationGraph::vertex_descriptor v = boost::target(ed, G);
//
//						cv::Point2f cu(tempCentroids.at<double>(G[u].leafID, 0), tempCentroids.at<double>(G[u].leafID, 1));
//						cv::Point2f cv(tempCentroids.at<double>(G[v].leafID, 0), tempCentroids.at<double>(G[v].leafID, 1));
//						cv::line(displayContours, cu, cv, cv::Scalar(255, 255, 255), 1);
//						++edgeCount;
//					}
//					std::cerr << edgeCount << std::endl;
//					cv::imshow("contours", displayContours);
////					cv::waitKey();
//				}

				if (k >= 1)
				{
					joinFrameGraphs(G, labels[labels.size()-2], labels.back(), k-1, k, tracker->flows2[k]);

					// Add rigid motion cliques
					if (motionClient.exists())
					{
						mps_msgs::ClusterRigidMotionsRequest req;
						req.flow_field.reserve(tracker->flows3[k].size());
						std::vector<size_t> valid_flow_to_all_flow_lookup;
						for (size_t f = 0; f<tracker->flows3[k].size(); ++f)
						{
							const auto& flow = tracker->flows3[k][f];
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
						std::cerr << tracker->flows3[k].size() << "->" << req.flow_field.size() << std::endl;

						mps_msgs::ClusterRigidMotionsResponse resp;

						bool success = motionClient.call(req, resp);
						if (success)
						{
							assert(resp.labels.size()==req.flow_field.size());
//						for (const auto l : resp.labels) { std::cerr << l << "\t"; } std::cerr << std::endl;

							std::map<int, std::vector<size_t>> flow_clusters;
							for (size_t f = 0; f<req.flow_field.size(); ++f)
							{
								flow_clusters[resp.labels[f]].push_back(f);
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
										flows2.push_back(tracker->flows2[k][f]);
									}

//									joinMotionCliques(G, labels[labels.size()-2], labels.back(), k-1, k, flows2);
								}
							}
						}

					}
				}
			}


			typedef std::map<VideoSegmentationGraph::vertex_descriptor, unsigned long> mapping_t;
			typedef boost::shared_ptr<mapping_t> vertex_component_map;
			typedef boost::filtered_graph<VideoSegmentationGraph, boost::function<bool(VideoSegmentationGraph::edge_descriptor)>, boost::function<bool(VideoSegmentationGraph::vertex_descriptor)> > ComponentGraph;

			vertex_component_map mapping = boost::make_shared<mapping_t>();
			size_t num_components = boost::connected_components(G, boost::associative_property_map<mapping_t>(*mapping));

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

//			std::vector<int> componentLookup(boost::num_vertices(G));
//			int num_components = boost::connected_components(G, &componentLookup[0]);
//			std::vector<VideoSegmentationGraph> Gs(num_components);
//			for (size_t node = 0; node < componentLookup.size(); ++node)
//			{
//				int seg = componentLookup[node];
//
//			}

			{
				visualizeVideoGraph(G, displays, centroids);
			}

			{
				Eigen::SparseMatrix<double> adj = getAdjacencySparse(G);
				mps_msgs::SegmentGraphRequest req;
				for (const auto& triplet : to_triplets(adj))
				{
					req.adjacency.row_index.push_back(triplet.row());
					req.adjacency.col_index.push_back(triplet.col());
					req.adjacency.value.push_back(triplet.value());
				}
				req.num_labels = NUM_OUTPUT_LABELS;
				req.algorithm = "dbscan";//"spectral";

				mps_msgs::SegmentGraphResponse resp;
				segmentClient.call(req, resp);
//				std::cerr << resp << std::endl;

				// Recolor image based on segments

				std::vector<cv::Scalar> segment_colors;
				cv::RNG rng(0);
				for (int i = 0; i < req.num_labels; ++i)
				{
					segment_colors.push_back(randomColor(rng));
				}
				std::vector<cv::Mat> video;
				for (const auto& label : labels)
				{
					video.emplace_back(cv::Mat(label.size(), CV_8UC3));
				}

				for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(G)))
				{
					const NodeProperties& np = G[vd];
					video[np.k].setTo(segment_colors[resp.labels[vd]], labels[np.k]==np.leafID);
				}


				cv::VideoWriter tracking("clustered.avi", CV_FOURCC('M', 'J', 'P', 'G'), 1, labels.front().size(), true);
				for (const auto& frame : video)
					tracking.write(frame);
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


			while (ros::ok())
			{
				cv::waitKey(10);
			}
			tracker->reset();
			ros::requestShutdown();
		}
		cv::waitKey(1);
		ros::Duration(1.0).sleep();
		cv::waitKey(1);
	}

	tracker.reset();
	segmentationClient.reset();
	segmentationPub.reset();
	return 0;
}