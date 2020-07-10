//
// Created by kunhuang on 6/27/20.
//
#include "mps_voxels/ExperimentDir.h"

const std::vector<std::string> ExperimentDir::checkpoints = {"0_measurement", "1_mixture", "2_estimate", "3_motion", "4_freespace_refined", "tracking_baseline"};
const size_t ExperimentDir::bestGuessIndex = 2;
