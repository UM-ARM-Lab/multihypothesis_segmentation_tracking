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

#include "mps_voxels/Ultrametric.h"
#include "mps_voxels/util/assert.h"

#include <ucm2hier.hpp> // NB: Contains function definitions, so can only be included once per project

Ultrametric::Ultrametric(const cv::Mat& ucm, const cv::Mat& labels)
{
	cv::Mat small_labels(labels.rows/2, labels.cols/2, CV_16UC1);
	for (int u = 1; u < labels.cols; u+=2)
	{
		for (int v = 1; v < labels.rows; v+=2)
		{
			small_labels.at<uint16_t>(v/2, u/2) = labels.at<uint16_t>(v, u);
		}
	}

	if (CV_MAT_DEPTH(ucm.type())!=CV_64FC1) { throw std::runtime_error("UCM must be of type 'float64'"); }
	if (CV_MAT_DEPTH(small_labels.type())!=CV_16UC1) { throw std::runtime_error("Labels must be of type 'uint16'"); }

	Eigen::Map<const Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> contour_map(
		ucm.ptr<double>(),
		ucm.rows, ucm.cols);
	Eigen::Map<const Eigen::Matrix<uint16_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> label_map(
		small_labels.ptr<uint16_t>(), small_labels.rows, small_labels.cols);


	std::cerr << label_map.all() << " ; " << cv::countNonZero(small_labels==0) << std::endl;
	std::cerr << label_map.rows() << ", " << contour_map.rows() << " ; " << label_map.cols() << ", "
	          << contour_map.cols() << std::endl;

	MPS_ASSERT(label_map.all());

	merge_tree = ucm2hier(contour_map, label_map.cast<label_type>());

	MPS_ASSERT(merge_tree.parent_labels.size()==merge_tree.n_regs-merge_tree.n_leaves);
	MPS_ASSERT(merge_tree.children_labels.size()==merge_tree.n_regs-merge_tree.n_leaves);

	for (label_type lbl = 1; lbl <= merge_tree.n_leaves; ++lbl) { label_to_index[lbl] = lbl-1; }
	// Construct an upward-pointing tree from the downward-pointing one
	std::set<label_type> all_leaf_children;
	for (label_type iter = 0; iter < merge_tree.children_labels.size(); ++iter)
	{
		label_to_index[merge_tree.parent_labels[iter]] = iter + merge_tree.n_leaves;
		for (label_type& lbl : merge_tree.children_labels[iter])
		{
			MPS_ASSERT(lbl <= merge_tree.n_regs);
			// Verify that this leaf has not been given another parent
			auto res = all_leaf_children.insert(lbl);
			MPS_ASSERT(res.second);
			EIGEN_UNUSED_VARIABLE(res);

			parent_tree[lbl] = merge_tree.parent_labels[iter];
		}
	}
}

double Ultrametric::getCost(const label_type lbl) const
{
	if (lbl == merge_tree.n_regs)
	{
		return 1.0;
	}
	else
	{
		return merge_tree.start_ths[label_to_index.at(lbl)];
	}
}

double Ultrametric::distance(const label_type a, const label_type b, const double maxDist) const
{
	label_type ancestor;
	std::set<int> i_ancestors, j_ancestors;
	label_type i_iter = a, j_iter = b;

	for (int iter = 0; iter < static_cast<int>(merge_tree.n_regs-merge_tree.n_leaves); ++iter) // max number of iters, should always terminate early
	{
		// Check if we've found an ancestor of j with i
		if (j_ancestors.find(i_iter) != j_ancestors.end())
		{
			ancestor = i_iter;
			return getCost(ancestor);
		}

		// Check if we've found an ancestor of i with j
		if (i_ancestors.find(j_iter) != i_ancestors.end())
		{
			ancestor = j_iter;
			return getCost(ancestor);
		}

		// Go up the tree in i
		i_ancestors.insert(i_iter);
		i_iter = parent_tree.at(i_iter);
		if (getCost(i_iter) > maxDist) { return -1; }
		if (merge_tree.n_regs == i_iter) { return 1.0; }

		// Go up the tree in j
		j_ancestors.insert(j_iter);
		j_iter = parent_tree.at(j_iter);
		if (getCost(j_iter) > maxDist) { return -1; }
		if (merge_tree.n_regs == j_iter) { return 1.0; }
	}
	throw std::runtime_error("Tree lookup failed.");
}

std::list<label_type> Ultrametric::getChildren(const merging_sequence& tree, const label_type n) const
{
	unsigned idx = label_to_index.at(n);
	if (idx < tree.n_leaves) { return {n}; }

	std::list<label_type> children;
	for (const label_type& child : tree.children_labels[idx-tree.n_leaves])
	{
		children.splice(children.end(), getChildren(tree, child));
	}
	return children;
}

std::map<label_type, std::vector<label_type>> Ultrametric::getCutClusters(const double threshold) const
{
	std::map<label_type, std::vector<label_type>> clusters;

	// For all nodes below the threshold with parents above the threshold
	for (const auto& lbl_pair : parent_tree)
	{
		if (getCost(lbl_pair.first) <= threshold)
		{
			if (getCost(lbl_pair.second) > threshold)
			{
				std::list<label_type> c = getChildren(merge_tree, lbl_pair.first);
				std::vector<label_type> children; children.reserve(c.size());
				children.insert(children.end(), c.begin(), c.end());
				for (const auto& child : children) { MPS_ASSERT(label_to_index.at(child) < merge_tree.n_leaves); EIGEN_UNUSED_VARIABLE(child); }
				clusters.insert({lbl_pair.first, children});
			}
		}
	}

	return clusters;
}
