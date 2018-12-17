//
// Created by arprice on 10/24/18.
//

#ifndef PROJECT_GRAPH_MATRIX_UTILS_HPP
#define PROJECT_GRAPH_MATRIX_UTILS_HPP

#include "mps_voxels/graph_matrix_utils.h"
#include "mps_voxels/video_graph.h"

template <typename Graph>
Eigen::MatrixXd getLaplacian(const Graph& graph)
{
	const int numCells = boost::num_vertices(graph);
	Eigen::MatrixXd laplacian = Eigen::MatrixXd::Zero(numCells, numCells);

	for (typename Graph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		size_t degree = boost::out_degree(vd, graph);

		// Degree Matrix
		laplacian(vd, vd) = degree;

		// Minus Adjacency Matrix
		for (typename Graph::edge_descriptor ed : make_range(boost::out_edges(vd, graph)))
		{
			typename Graph::vertex_descriptor u = boost::target(ed, graph);
			laplacian(vd, u) = -graph[ed].affinity;
		}
	}

	return laplacian;
}

template <typename Graph>
Eigen::MatrixXd getLaplacianNormalized(const Graph& graph)
{
	const int numCells = boost::num_vertices(graph);

	Eigen::MatrixXd laplacian = Eigen::MatrixXd::Zero(numCells, numCells);
	Eigen::VectorXd d(numCells);

	for (typename Graph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		size_t degree = boost::out_degree(vd, graph);

		// Degree Matrix
		laplacian(vd, vd) = degree;

		// Degree Matrix Normalizer
		d(vd) = 1.0/sqrt(static_cast<double>(degree));

		// Minus Adjacency Matrix
		for (typename Graph::edge_descriptor ed : make_range(boost::out_edges(vd, graph)))
		{
			typename Graph::vertex_descriptor u = boost::target(ed, graph);
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

	for (typename Graph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		size_t degree = boost::out_degree(vd, graph);

		// Degree Matrix
		triplets.emplace_back(Tripletd(vd, vd, degree));

		// Minus Adjacency Matrix
		for (typename Graph::edge_descriptor ed : make_range(boost::out_edges(vd, graph)))
		{
			typename Graph::vertex_descriptor u = boost::target(ed, graph);
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

	for (typename Graph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		size_t degree = boost::out_degree(vd, graph);

		// Degree Matrix
		triplets.emplace_back(Tripletd(vd, vd, degree));

		// Degree Matrix Normalizer
		d(vd) = 1.0/sqrt(static_cast<double>(degree));

		// Minus Adjacency Matrix
		for (typename Graph::edge_descriptor ed : make_range(boost::out_edges(vd, graph)))
		{
			typename Graph::vertex_descriptor u = boost::target(ed, graph);
			triplets.emplace_back(Tripletd(vd, u, -graph[ed].affinity));
		}
	}

	Eigen::SparseMatrix<double> laplacian(numCells, numCells);
	laplacian.setFromTriplets(triplets.begin(), triplets.end());

	return d.asDiagonal() * laplacian * d.asDiagonal();
}

template <typename Graph>
Eigen::SparseMatrix<double> getAdjacencySparse(const Graph& graph)
{
	const int numCells = boost::num_vertices(graph);

	typedef Eigen::Triplet<double> Tripletd;
	std::vector<Tripletd> triplets;
	triplets.reserve(2*numCells);

	for (typename Graph::vertex_descriptor vd : make_range(boost::vertices(graph)))
	{
		// Adjacency Matrix
		for (typename Graph::edge_descriptor ed : make_range(boost::out_edges(vd, graph)))
		{
			typename Graph::vertex_descriptor u = boost::target(ed, graph);
			triplets.emplace_back(Tripletd(vd, u, graph[ed].affinity)); // u->v is added when vd = u
		}
	}

	Eigen::SparseMatrix<double> adjacency(numCells, numCells);
	adjacency.setFromTriplets(triplets.begin(), triplets.end());

	return adjacency;
}

template <typename Scalar>
std::vector<Eigen::Triplet<Scalar>> to_triplets(const Eigen::SparseMatrix<Scalar> & M)
{
	std::vector<Eigen::Triplet<Scalar>> v;
	for(int i = 0; i < M.outerSize(); i++)
		for(typename Eigen::SparseMatrix<Scalar>::InnerIterator it(M,i); it; ++it)
			v.emplace_back(it.row(),it.col(),it.value());
	return v;
}

#endif // PROJECT_GRAPH_MATRIX_UTILS_HPP
