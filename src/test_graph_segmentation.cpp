//
// Created by arprice on 10/22/18.
//

#include <Eigen/Eigenvalues>
#include <Eigen/SparseCore>
//#include <Spectra/GenEigsSolver.h>
//#include <Spectra/MatOp/SparseGenMatProd.h>
//#include <Spectra/SymEigsSolver.h>
//#include <Spectra/MatOp/SparseSymMatProd.h>

#include "mps_voxels/map_graph.h"
#include "mps_voxels/graph_matrix_utils.h"
#include <mps_msgs/SegmentGraph.h>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/connected_components.hpp>
#include <boost/graph/graphviz.hpp>

#include <ros/ros.h>


template <class WeightMap>
class edge_writer {
public:
	edge_writer(WeightMap w) : wm(w) {}
	template <class Edge>
	void operator()(std::ostream &out, const Edge& e) const {
		out << "[penwidth=\"" << wm[e]*4.0 << "\"]";
	}
private:
	WeightMap wm;
};

template <class WeightMap>
inline edge_writer<WeightMap>
make_edge_writer(WeightMap w) {
	return edge_writer<WeightMap>(w);
}

std::string node_name(const NodeProperties& np) {return "f" + std::to_string(np.k) + "c" + std::to_string(np.leafID);}

void print(std::ostream& out, const VideoSegmentationGraph& G)
{
	out << "graph G {\n";

	std::map<int, std::vector<VideoSegmentationGraph::vertex_descriptor>> frames;
	for (VideoSegmentationGraph::vertex_descriptor vd : make_range(boost::vertices(G)))
	{
		frames[G[vd].k].push_back(vd);
	}

	for (const auto& frame : frames)
	{
		out << "subgraph cluster_" << frame.first << " {\n";
		out << "label=\"Frame " << frame.first << "\";\n";
		for (const VideoSegmentationGraph::vertex_descriptor vd : frame.second)
		{
			const NodeProperties& np = G[vd];
			out << node_name(np) << " [label=\"" << np.leafID << "\"];\n";
		}
		out << "}\n";
	}

	for (VideoSegmentationGraph::edge_descriptor ed : make_range(boost::edges(G)))
	{
		VideoSegmentationGraph::vertex_descriptor u = boost::source(ed, G);
		VideoSegmentationGraph::vertex_descriptor v = boost::target(ed, G);

		const EdgeProperties& ep = G[ed];
		const NodeProperties& npu = G[u];
		const NodeProperties& npv = G[v];
		out << node_name(npu) << "--" << node_name(npv) << " [penwidth=\"" << 4.0*ep.affinity << "\"];\n";
	}

	out << "}\n";
}

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "test_graph_segmentation");
	ros::NodeHandle nh, pnh("~");

	ros::ServiceClient segmentClient = nh.serviceClient<mps_msgs::SegmentGraph>("/segment_graph");
	if (!segmentClient.waitForExistence(ros::Duration(3)))
	{
		ROS_ERROR("Segmentation server not connected.");
		return -1;
	}

	VideoSegmentationGraph G;

	std::map<std::pair<int, int>, VideoSegmentationGraph::vertex_descriptor> segmentToNode;
	std::vector<NodeProperties> nps{NodeProperties({0, 0}), NodeProperties({0, 1}), NodeProperties({0, 2}), NodeProperties({1, 0}), NodeProperties({1, 1})};
	for (const auto& np : nps)
	{
		// Add leaf i to graph
		// NB: labels start from 1, as 0 is background, so tree[0] = labels[1]
		VideoSegmentationGraph::vertex_descriptor v = boost::add_vertex(np, G);
		segmentToNode.insert({{np.k, np.leafID}, v});
	}

	boost::add_edge(segmentToNode[{0, 0}], segmentToNode[{0, 1}], {1.0/1.0}, G);
	boost::add_edge(segmentToNode[{0, 0}], segmentToNode[{0, 2}], {1.0/10.0}, G);
	boost::add_edge(segmentToNode[{0, 1}], segmentToNode[{0, 2}], {1.0/10.0}, G);
	boost::add_edge(segmentToNode[{1, 0}], segmentToNode[{1, 1}], {1.0/15.0}, G);
	boost::add_edge(segmentToNode[{0, 2}], segmentToNode[{1, 1}], {1.0/2.0}, G);
	boost::add_edge(segmentToNode[{0, 0}], segmentToNode[{1, 0}], {1.0/2.0}, G);
	boost::add_edge(segmentToNode[{0, 1}], segmentToNode[{1, 0}], {1.0/2.0}, G);

	print(std::cerr, G);//, boost::make_label_writer(boost::get(&NodeProperties::leafID, G)), make_edge_writer(boost::get(&EdgeProperties::affinity, G)));

	{
		Eigen::MatrixXd laplacian = getLaplacianNormalized(G);
		const int numCells = laplacian.rows();
		std::cerr << "Segmenting subgraph with " << numCells << " vertices (out of " << boost::num_vertices(G) << ")."
		          << std::endl;
		Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> solver;
		solver.compute(laplacian, Eigen::ComputeEigenvectors);
		Eigen::VectorXd eigenvalues = solver.eigenvalues();
		Eigen::MatrixXd eigenvectors = solver.eigenvectors();
		std::cerr << eigenvalues.transpose() << std::endl;
		std::cerr << eigenvectors << std::endl;
	}

//	{
//		Eigen::SparseMatrix<double> laplacian = getLaplacianSparseNormalized(G);
//		using SparseMatProd = Spectra::SparseGenMatProd<double>; // Spectra::SparseSymMatProd<double>
//		SparseMatProd op(laplacian);
//		Spectra::GenEigsSolver<double, Spectra::LARGEST_MAGN, SparseMatProd> eigs(&op, 3, 5);
//		eigs.init();
//		int nconv = eigs.compute();
//		if (eigs.info() == Spectra::SUCCESSFUL)
//		{
//			std::cerr << nconv << "\n" << eigs.eigenvectors() << std::endl;
//		}
//		else
//		{
//			ROS_WARN("Failed to compute eigenvectors. Clusterign aborted.");
//		}
//	}

	{
		Eigen::SparseMatrix<double> adj = getAdjacencySparse(G);
		mps_msgs::SegmentGraphRequest req;
		for (const auto& triplet : to_triplets(adj))
		{
			std::cerr << triplet.row() << "," << triplet.col() << "," << triplet.value() << std::endl;
			req.adjacency.row_index.push_back(triplet.row());
			req.adjacency.col_index.push_back(triplet.col());
			req.adjacency.value.push_back(triplet.value());
		}
		req.num_labels = 2;

		for (const std::string& alg : std::vector<std::string>{"spectral", "dbscan"})
		{
			req.algorithm = alg;

			mps_msgs::SegmentGraphResponse resp;
			segmentClient.call(req, resp);
			std::cerr << resp << std::endl;

		}
	}

	return 0;
}