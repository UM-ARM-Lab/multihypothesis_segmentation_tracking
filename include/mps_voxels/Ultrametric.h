//
// Created by arprice on 10/24/18.
//

#ifndef PROJECT_ULTRAMETRIC_H
#define PROJECT_ULTRAMETRIC_H

#include <containers.hpp>
#include <opencv2/core.hpp>

struct Ultrametric
{
	merging_sequence merge_tree;
	std::map<label_type, label_type> parent_tree;
	std::map<label_type, unsigned> label_to_index;

	Ultrametric(cv::Mat& ucm, cv::Mat& labels);

	double getCost(const label_type lbl) const;

	// returns -1 if distance is larger than maxDist
	double distance(const label_type a, const label_type b, const double maxDist = 0.5) const;

	std::list<label_type> getChildren(const merging_sequence& tree, const label_type n) const;

	std::map<label_type, std::vector<label_type>> getCutClusters(const double threshold) const;
};


#endif // PROJECT_ULTRAMETRIC_H
