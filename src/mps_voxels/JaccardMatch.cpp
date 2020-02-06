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

#include "mps_voxels/JaccardMatch.h"
#include "mps_voxels/AABB.h"
#include "mps_voxels/hungarian.hpp"
#include <boost/bimap.hpp>
#include <utility>

namespace mps
{

JaccardMatch::JaccardMatch(const cv::Mat& labels1, const cv::Mat& labels2)
{
	using LabelT = uint16_t;

	if (labels1.channels() > 1 || labels1.type() != CV_16U || labels2.channels() > 1 || labels2.type() != CV_16U)
	{
		throw std::logic_error("jaccard() !!! Only works with CV_16U 1-channel Mats");
	}

	boxes1 = getBBoxes(labels1);
	boxes2 = getBBoxes(labels2);

	int count = 0;
	boost::bimap<LabelT, int> lblIndex1; for (const auto i : boxes1) { lblIndex1.insert({i.first, count++}); }
	count = 0;
	boost::bimap<LabelT, int> lblIndex2; for (const auto j : boxes2) { lblIndex2.insert({j.first, count++}); }

	double smoothing = 1.0;

	size_t dSize = std::max(boxes1.size(), boxes2.size());
	D = Eigen::MatrixXd::Zero(dSize, dSize);

	std::map<LabelT, int> jSizes;

	for (const auto i : boxes1)
	{
		cv::Mat mask1 = (labels1 == i.first);
		int count1 = cv::countNonZero(mask1);
		for (const auto j : boxes2)
		{
			double iou = 0;
			// Prune actual comparisons by starting with bounding boxes
			if (intersect(i.second, j.second))
			{
				cv::Mat mask2 = (labels2 == j.first);

				int count2;
				const auto iter = jSizes.find(j.first);
				if (iter == jSizes.end())
				{
					count2 = cv::countNonZero(mask2);
					jSizes.insert({j.first, count2});
				}
				else
				{
					count2 = iter->second;
				}

				cv::Mat intersectionMask = (mask1 & mask2);

				int intersectionCount = cv::countNonZero(intersectionMask);
				int unionCount = count1 + count2 - intersectionCount;
//			    cv::Mat unionMask = (mask1 | mask2); // Optimized to be (|A|+|B|-|AB|)
				iou = (intersectionCount + smoothing) /
				      static_cast<double>(unionCount + smoothing);
			}
			D(lblIndex1.left.at(i.first), lblIndex2.left.at(j.first)) = -iou;
		}
	}

	// Compute the optimal assignment of patches
	Hungarian H(D);
	const auto& A = H.getAssignment();
	boost::bimap<LabelT, LabelT> matches;
	for (size_t i = 0; i < A.size(); ++i)
	{
		if (i >= boxes1.size()) { continue; }
		if (A[i] >= (int)boxes2.size()) { continue; }
		matches.insert({lblIndex1.right.at(i), lblIndex2.right.at(A[i])});
	}

	match = {-H.getSolutionCost(), matches};
}

}
