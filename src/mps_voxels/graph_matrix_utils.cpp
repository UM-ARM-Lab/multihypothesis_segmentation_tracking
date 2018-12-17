//
// Created by arprice on 10/24/18.
//

#include "mps_voxels/graph_matrix_utils_impl.hpp"
#include "mps_voxels/video_graph.h"

template
Eigen::MatrixXd getLaplacian<VideoSegmentationGraph>(const VideoSegmentationGraph& graph);

template
Eigen::MatrixXd getLaplacianNormalized<VideoSegmentationGraph>(const VideoSegmentationGraph& graph);

template
Eigen::SparseMatrix<double> getLaplacianSparse<VideoSegmentationGraph>(const VideoSegmentationGraph& graph);

template
Eigen::SparseMatrix<double> getLaplacianSparseNormalized<VideoSegmentationGraph>(const VideoSegmentationGraph& graph);

template
Eigen::SparseMatrix<double> getAdjacencySparse<VideoSegmentationGraph>(const VideoSegmentationGraph& graph);

template
std::vector<Eigen::Triplet<double>> to_triplets<double>(const Eigen::SparseMatrix<double> & M);