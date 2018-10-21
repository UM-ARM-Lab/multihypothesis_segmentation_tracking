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

#include <visualization_msgs/MarkerArray.h>
#include <depth_image_proc/depth_traits.h>
#include <image_geometry/pinhole_camera_model.h>

#include <ucm2hier.hpp>

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

template <typename Graph>
Eigen::MatrixXd getLaplacian(const Graph& graph)
{
	const int numCells = boost::num_vertices(graph);
	Eigen::MatrixXd laplacian = Eigen::MatrixXd::Zero(numCells, numCells);

	for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		size_t degree = boost::out_degree(vd, graph);

		// Degree Matrix
		laplacian(vd, vd) = degree;

		// Minus Adjacency Matrix
		for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::out_edges(vd, graph)))
		{
			VideoSegmentationGraph::vertex_descriptor u = boost::target(ed, graph);
			laplacian(vd, u) = -graph[ed].affinity;
		}
	}

	return laplacian;
}

template <typename Graph>
Eigen::MatrixXd getLaplacianNormalized(const Graph& graph)
{
	int numCells = 0;
	for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		numCells++;
	}

	Eigen::MatrixXd laplacian = Eigen::MatrixXd::Zero(numCells, numCells);
	Eigen::VectorXd d(numCells);

	for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		size_t degree = boost::out_degree(vd, graph);

		// Degree Matrix
		laplacian(vd, vd) = degree;

		// Degree Matrix Normalizer
		d(vd) = 1.0/sqrt(static_cast<double>(degree));

		// Minus Adjacency Matrix
		for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::out_edges(vd, graph)))
		{
			VideoSegmentationGraph::vertex_descriptor u = boost::target(ed, graph);
			laplacian(vd, u) = -graph[ed].affinity;
		}
	}

	return d.asDiagonal() * laplacian * d.asDiagonal();
}

template <typename Graph>
Eigen::SparseMatrix<double> getLaplacianSparse(const Graph& graph)
{
	const int numCells = boost::num_vertices(graph);

	typedef Eigen::Triplet<double> Tripletd;
	std::vector<Tripletd> triplets;
	triplets.reserve(numCells);

	for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		size_t degree = boost::out_degree(vd, graph);

		// Degree Matrix
		triplets.emplace_back(Tripletd(vd, vd, degree));

		// Minus Adjacency Matrix
		for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::out_edges(vd, graph)))
		{
			VideoSegmentationGraph::vertex_descriptor u = boost::target(ed, graph);
			triplets.emplace_back(Tripletd(vd, u, -graph[ed].affinity)); // u->v is added when vd = u
		}
	}

	Eigen::SparseMatrix<double> laplacian(numCells, numCells);
	laplacian.setFromTriplets(triplets.begin(), triplets.end());

	return laplacian;
}

template <typename Graph>
Eigen::SparseMatrix<double> getLaplacianSparseNormalized(const Graph& graph)
{
	const int numCells = boost::num_vertices(graph);

	typedef Eigen::Triplet<double> Tripletd;
	std::vector<Tripletd> triplets;
	triplets.reserve(numCells);

	Eigen::VectorXd d(numCells);

	for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		size_t degree = boost::out_degree(vd, graph);

		// Degree Matrix
		triplets.emplace_back(Tripletd(vd, vd, degree));

		// Degree Matrix Normalizer
		d(vd) = 1.0/sqrt(static_cast<double>(degree));

		// Minus Adjacency Matrix
		for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::out_edges(vd, graph)))
		{
			VideoSegmentationGraph::vertex_descriptor u = boost::target(ed, graph);
			triplets.emplace_back(Tripletd(vd, u, -graph[ed].affinity));
		}
	}

	Eigen::SparseMatrix<double> laplacian(numCells, numCells);
	laplacian.setFromTriplets(triplets.begin(), triplets.end());

	return d.asDiagonal() * laplacian * d.asDiagonal();
}

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

bool getLowestCommonAncestor(const merging_sequence& tree, const int i, const int j, int& ancestor, const double maxDist = 0.5)
{
	std::set<int> i_ancestors, j_ancestors;
	int i_iter = i, j_iter = j;

	for (int iter = 0; iter < static_cast<int>(tree.parent_labels.size()); ++iter) // max number of iters, should always terminate early
	{
		// Check if we've found an ancestor of j with i
		if (j_ancestors.find(i_iter) != j_ancestors.end())
		{
			ancestor = i_iter;
			return true;
		}

		// Check if we've found an ancestor of i with j
		if (i_ancestors.find(j_iter) != i_ancestors.end())
		{
			ancestor = j_iter;
			return true;
		}

		// Go up the tree in i
		i_ancestors.insert(i_iter);
		assert(i_iter >= 0);
		assert(i_iter < tree.parent_labels.size());
		i_iter = static_cast<int>(tree.parent_labels[i_iter]);
		assert(i_iter >= 0);
		assert(i_iter < tree.parent_labels.size());
		assert(i_iter >= tree.n_leaves);
		if (tree.start_ths[i_iter] > maxDist)
		{
			return false;
		}

		// Go up the tree in j
		j_ancestors.insert(j_iter);
		assert(j_iter >= 0);
		assert(j_iter < tree.parent_labels.size());
		j_iter = static_cast<int>(tree.parent_labels[j_iter]);
		assert(j_iter >= 0);
		assert(j_iter < tree.parent_labels.size());
		assert(j_iter >= tree.n_leaves);
		if (tree.start_ths[j_iter] > maxDist)
		{
			return false;
		}
	}
	return false;
}

std::list<label_type> getChildren(const merging_sequence& tree, const label_type n)
{
	if (n < tree.n_leaves) { return {n}; }

	std::list<label_type> children;
	for (const label_type& child : tree.children_labels[n-tree.n_leaves])
	{
		children.splice(children.end(), getChildren(tree, child));
	}
	return children;
}

void getCutClusters(const merging_sequence& tree, const double threshold, std::map<label_type, std::vector<label_type>>& clusters)
{
	assert(tree.parent_labels.size() == tree.start_ths.size());
	assert(tree.parent_labels.size() == tree.n_regs);

	// For all nodes below the threshold with parents above the threshold
	for (label_type iter = 0; iter < tree.parent_labels.size(); ++iter)
	{
		if (tree.start_ths[iter] <= threshold)
		{
			std::cerr << "maybe?: " << iter << " -> " << tree.parent_labels[iter] << " = " << tree.start_ths[tree.parent_labels[iter]] << std::endl;
			if (tree.start_ths[tree.parent_labels[iter]] > threshold)
			{
				std::cerr << "cut: " << iter << std::endl;
				std::list<label_type> c = getChildren(tree, iter);
				std::vector<label_type> children; children.reserve(c.size());
				children.insert(children.end(), c.begin(), c.end());
				clusters.insert({iter, children});
			}
		}
	}
}

void addToGraph(VideoSegmentationGraph& G, cv::Mat& ucm, cv::Mat& labels, const int k)
{
	if (CV_MAT_DEPTH(ucm.type())!=CV_64F) { throw std::runtime_error("UCM must be of type 'float64'"); }
	if (CV_MAT_DEPTH(labels.type())!=CV_16U) { throw std::runtime_error("Labels must be of type 'uint16'"); }
	const double EDGE_THRESHOLD = 0.2;

	Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> contour_map(ucm.ptr<double>(),
	                                                                                               ucm.rows, ucm.cols);
	Eigen::Map<Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> label_map(
		labels.ptr<uint16_t>(), labels.rows, labels.cols);
//	Eigen::Map<Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>, Eigen::Unaligned, Eigen::Stride<Eigen::Dynamic,2>>
//		label_map(labels.ptr<uint16_t>()+1, labels.rows/2, labels.cols/2, Eigen::Stride<Eigen::Dynamic,2>(labels.cols, 2));

	std::cerr << label_map.all() << " ; " << cv::countNonZero(labels==0) << std::endl;
	std::cerr << label_map.rows() << ", " << contour_map.rows() << " ; " << label_map.cols() << ", "
	          << contour_map.cols() << std::endl;

//	cv::Mat display_labels = (labels == 0);
//	display_labels.convertTo(display_labels, CV_8UC1);
//	cv::imshow("labels", display_labels);
//	cv::waitKey();

	merging_sequence tree = ucm2hier(contour_map, label_map.cast<label_type>());

	assert(tree.parent_labels.size()==tree.n_regs-tree.n_leaves);
	assert(tree.children_labels.size()==tree.n_regs-tree.n_leaves);

	// ucm2hier assumes data is prepped for MATLAB, so it is 1-indexed
//	for (label_type& lbl : tree.parent_labels)
//	{
//		--lbl;
//		assert(lbl < tree.n_regs); // top pointer to root?
//		assert(lbl >= tree.n_leaves);
//	}
	for (size_t iter = 0; iter<tree.children_labels.size(); ++iter)
	{
		for (label_type& lbl : tree.children_labels[iter])
		{
			--lbl;
			assert(lbl<tree.n_regs);
			assert(lbl>=0);
		}
	}

	// Reconstruct parent pointers for leaf nodes
	std::vector<label_type> parents(tree.n_regs);
	std::iota(parents.begin(), parents.end(), 0);
	std::set<label_type> all_leaf_children;
	for (size_t iter = 0; iter<tree.children_labels.size(); ++iter)
	{
		assert(tree.parent_labels[iter]-1==iter+tree.n_leaves);
		for (const label_type lbl : tree.children_labels[iter])
		{
			auto res = all_leaf_children.insert(lbl);
			assert(res.second);
			label_type parent = iter+tree.n_leaves;
			assert(parent>=0);
			assert(parent<tree.n_regs);
			parents[lbl] = parent;
		}
	}
//	leaf_parents.reserve(tree.n_regs);
//	leaf_parents.insert(leaf_parents.end(), tree.parent_labels.begin(), tree.parent_labels.end());
	tree.parent_labels = parents;

	// Sanity check that all parents are valid non-leaf nodes
	for (label_type iter = 0; iter<tree.parent_labels.size(); ++iter)
	{
		const auto& lbl = tree.parent_labels[iter];
		assert(lbl<tree.n_regs); // top pointer to root?
		assert(lbl>=tree.n_leaves || lbl==iter);
	}

	std::map<std::pair<int, int>, VideoSegmentationGraph::vertex_descriptor> segmentToNode;

	for (int i = 0; i<static_cast<int>(tree.n_leaves); ++i)
	{
		// Add leaf i to graph
		// NB: labels start from 1, as 0 is background, so tree[0] = labels[1]
		VideoSegmentationGraph::vertex_descriptor v = boost::add_vertex(NodeProperties({k, i+1}), G);
		segmentToNode.insert({{k, i}, v});
	}

	std::map<label_type, std::vector<label_type>> preclusters;
	getCutClusters(tree, EDGE_THRESHOLD, preclusters);

	for (const auto& precluster : preclusters)
	{
		for (int i = 0; i<static_cast<int>(precluster.second.size()); ++i)
		{
			for (int j = i+1; j<static_cast<int>(precluster.second.size()); ++j)
			{
				int ancestor = -1;
				label_type p = precluster.second[i];
				label_type q = precluster.second[j];
				if (getLowestCommonAncestor(tree, p, q, ancestor, EDGE_THRESHOLD))
				{
					double dist = tree.start_ths[ancestor];

					VideoSegmentationGraph::vertex_descriptor s = segmentToNode[{k, p}];
					VideoSegmentationGraph::vertex_descriptor t = segmentToNode[{k, q}];
					boost::add_edge(s, t, {1.0/(dist+1e-4)}, G);
				}
			}
		}
	}
}

void joinFrameGraphs(VideoSegmentationGraph& G, const cv::Mat& iLabels, const cv::Mat& jLabels, const int i, const int j, const Tracker::Flow2D& flow)
{
	const double weight = 1/0.02;
	std::map<std::pair<int, int>, VideoSegmentationGraph::vertex_descriptor> segmentToNode;
	for (const std::pair<cv::Point2f, cv::Point2f>& flowVec : flow)
	{
		int iLabel = iLabels.at<uint16_t>(static_cast<cv::Point>(flowVec.first/2));
		int jLabel = jLabels.at<uint16_t>(static_cast<cv::Point>(flowVec.second/2));

		auto res = boost::add_edge(segmentToNode[{i, iLabel}], segmentToNode[{j, jLabel}], G);
		EdgeProperties& eProps = G[res.first];
		eProps.affinity += weight;
	}
}


int main(int argc, char* argv[])
{
	ros::init(argc, argv, "flow");
	ros::NodeHandle nh, pnh("~");

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
	std::vector<cv_bridge::CvImagePtr> segmentations;
	VideoSegmentationGraph G;

	while (ros::ok())
	{
		if (tracker->rgb_buffer.size() == tracker->MAX_BUFFER_LEN)
		{
			tracker->track(step);
			for (int i = 0; i < static_cast<int>(tracker->MAX_BUFFER_LEN)-step; i+=step)
			{
				cv::Rect roi(500, 500,
				             1000, 500);
				int pyrScale = 2;
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
				cv::Mat silly_labels_small(silly_ucm.rows/2, silly_ucm.cols/2, CV_16UC1);
				// Not sure if necessary...
				for (int u = 0; u < silly_ucm.cols; u+=2)
				{
					for (int v = 0; v < silly_ucm.rows; v+=2)
					{
						silly_ucm.at<double>(v, u) = 1.0;
					}
				}
				cv::connectedComponentsWithStats(silly_ucm == 0, tempLabels, stats, tempCentroids, 8, CV_16U);
				for (int u = 1; u < tempLabels.cols; u+=2)
				{
					for (int v = 1; v < tempLabels.rows; v+=2)
					{
						silly_labels_small.at<uint16_t>(v/2, u/2) = tempLabels.at<uint16_t>(v, u);
					}
				}

				labels.push_back(tempLabels);
				centroids.push_back(tempCentroids);

				double alpha = 0.75;
				cv::Mat labelColorsMap = colorByLabel(segmentations.back()->image);
				labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*rgb_cropped;
				cv::imshow("segmentation", labelColorsMap);
				cv::waitKey(1);

//				cv::Mat silliness;
//				silly_labels_small.convertTo(silly_labels_small, CV_8UC1);
//				cv::imshow("labels", silly_labels_small);

				cv::Mat displayLabels;
				tempLabels.convertTo(displayLabels, CV_8UC1);
				cv::imshow("labels", displayLabels);

				cv::Mat tempContours1, displayContours;
				double maxVal;
				cv::minMaxLoc(ucm->image, nullptr, &maxVal);
				ucm->image.convertTo(tempContours1, CV_8UC1, 255.0/maxVal);
				cv::applyColorMap(tempContours1, displayContours, cv::COLORMAP_JET); // COLORMAP_HOT
				cv::imshow("contours", displayContours);
				cv::waitKey(1);

				if (segmentationPub->getNumSubscribers() > 0)
				{
					segmentationPub->publish(cv_bridge::CvImage(tracker->cameraModel.cameraInfo().header, "bgr8", labelColorsMap).toImageMsg());
				}

				addToGraph(G, ucms.back(), silly_labels_small, i/step);

				if (i == 0)
				{
					int edgeCount = 0;
					for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::edges(G)))
					{
						VideoSegmentationGraph::vertex_descriptor u = boost::source(ed, G);
						VideoSegmentationGraph::vertex_descriptor v = boost::target(ed, G);

						cv::Point2f cu(tempCentroids.at<double>(G[u].leafID, 0), tempCentroids.at<double>(G[u].leafID, 1));
						cv::Point2f cv(tempCentroids.at<double>(G[v].leafID, 0), tempCentroids.at<double>(G[v].leafID, 1));
						cv::line(displayContours, cu, cv, cv::Scalar(255, 255, 255), 1);
						++edgeCount;
					}
					std::cerr << edgeCount << std::endl;
					cv::imshow("contours", displayContours);
					cv::waitKey();
				}

				if (i >= 1)
				{
					joinFrameGraphs(G, labels[labels.size()-2], labels.back(), i/step-1, i/step, tracker->flows2[i/step]);
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
				Eigen::MatrixXd laplacian = getLaplacianNormalized(G);
				const int numCells = laplacian.rows();
				std::cerr << "Segmenting subgraph with " << numCells << " vertices (out of " << boost::num_vertices(G) << ")." << std::endl;
				Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver;
				solver.compute(laplacian, Eigen::ComputeEigenvectors);
				Eigen::VectorXd eigenvalues = solver.eigenvalues();
				Eigen::MatrixXd eigenvectors = solver.eigenvectors();

			}

//			std::vector<int> componentLookup(boost::num_vertices(G));
//			int num_components = boost::connected_components(G, &componentLookup[0]);
//			std::vector<VideoSegmentationGraph> Gs(num_components);
//			for (size_t node = 0; node < componentLookup.size(); ++node)
//			{
//				int seg = componentLookup[node];
//
//			}



//			Eigen::SparseMatrix<double> laplacian = getLaplacianSparseNormalized(G);
//			using SparseMatProd = Spectra::SparseSymMatProd<double>; // Spectra::SparseGenMatProd<double>
//			SparseMatProd op(laplacian);
//			Spectra::SymEigsSolver<double, Spectra::LARGEST_MAGN, SparseMatProd> eigs(&op, 30, 2*30+1);
//			eigs.init();
//			int nconv = eigs.compute();
//			if (eigs.info() == Spectra::SUCCESSFUL)
//			{
//				std::cerr << nconv << "\n" << eigs.eigenvectors() << std::endl;
//			}
//			else
//			{
//				ROS_WARN("Failed to compute eigenvectors. Clusterign aborted.");
//
//				tracker->reset();
//				continue;
//			}


			cv::waitKey();
			tracker->reset();
		}
		cv::waitKey(1);
		ros::Duration(1.0).sleep();
		cv::waitKey(1);
	}

	return 0;
}