//
// Created by arprice on 10/24/18.
//

#include "mps_voxels/graph_matrix_utils_impl.hpp"
#include "mps_voxels/video_graph.h"

template
Eigen::MatrixXd getLaplacian<VideoSegmentationGraph<SEGMENT_TYPE::UCM>>(const VideoSegmentationGraph<SEGMENT_TYPE::UCM>& graph);

template
Eigen::MatrixXd getLaplacianNormalized<VideoSegmentationGraph<SEGMENT_TYPE::UCM>>(const VideoSegmentationGraph<SEGMENT_TYPE::UCM>& graph);

template
Eigen::SparseMatrix<double> getLaplacianSparse<VideoSegmentationGraph<SEGMENT_TYPE::UCM>>(const VideoSegmentationGraph<SEGMENT_TYPE::UCM>& graph);

template
Eigen::SparseMatrix<double> getLaplacianSparseNormalized<VideoSegmentationGraph<SEGMENT_TYPE::UCM>>(const VideoSegmentationGraph<SEGMENT_TYPE::UCM>& graph);

template
Eigen::SparseMatrix<double> getAdjacencySparse<VideoSegmentationGraph<SEGMENT_TYPE::UCM>>(const VideoSegmentationGraph<SEGMENT_TYPE::UCM>& graph);


template
std::vector<Eigen::Triplet<double>> to_triplets<double>(const Eigen::SparseMatrix<double> & M);



template
Eigen::MatrixXd getLaplacian<VideoSegmentationGraph<SEGMENT_TYPE::BODY>>(const VideoSegmentationGraph<SEGMENT_TYPE::BODY>& graph);

template
Eigen::MatrixXd getLaplacianNormalized<VideoSegmentationGraph<SEGMENT_TYPE::BODY>>(const VideoSegmentationGraph<SEGMENT_TYPE::BODY>& graph);

template
Eigen::SparseMatrix<double> getLaplacianSparse<VideoSegmentationGraph<SEGMENT_TYPE::BODY>>(const VideoSegmentationGraph<SEGMENT_TYPE::BODY>& graph);

template
Eigen::SparseMatrix<double> getLaplacianSparseNormalized<VideoSegmentationGraph<SEGMENT_TYPE::BODY>>(const VideoSegmentationGraph<SEGMENT_TYPE::BODY>& graph);

template
Eigen::SparseMatrix<double> getAdjacencySparse<VideoSegmentationGraph<SEGMENT_TYPE::BODY>>(const VideoSegmentationGraph<SEGMENT_TYPE::BODY>& graph);
