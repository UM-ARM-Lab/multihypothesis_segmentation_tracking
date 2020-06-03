//
// Created by arprice on 10/24/18.
//

#ifndef MPS_GRAPH_MATRIX_UTILS_H
#define MPS_GRAPH_MATRIX_UTILS_H

#include <Eigen/Core>
#include <Eigen/SparseCore>
#include <boost/graph/adjacency_list.hpp>

template <typename Graph>
Eigen::MatrixXd getLaplacian(const Graph& graph);

template <typename Graph>
Eigen::MatrixXd getLaplacianNormalized(const Graph& graph);

template <typename Graph>
Eigen::SparseMatrix<double> getLaplacianSparse(const Graph& graph);

template <typename Graph>
Eigen::SparseMatrix<double> getLaplacianSparseNormalized(const Graph& graph);

template <typename Graph>
Eigen::SparseMatrix<double> getAdjacencySparse(const Graph& graph);

template <typename Scalar>
std::vector<Eigen::Triplet<Scalar>> to_triplets(const Eigen::SparseMatrix<Scalar> & M);

#endif // MPS_GRAPH_MATRIX_UTILS_H
