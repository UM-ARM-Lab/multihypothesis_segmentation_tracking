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

#ifndef MPS_GRAPH_MATRIX_UTILS_HPP
#define MPS_GRAPH_MATRIX_UTILS_HPP

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

#endif // MPS_GRAPH_MATRIX_UTILS_HPP
