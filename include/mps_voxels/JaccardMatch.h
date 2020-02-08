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

#ifndef SRC_JACCARDMATCH_H
#define SRC_JACCARDMATCH_H

#include "mps_voxels/AABB.h"

#include <boost/bimap.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Core>
#include <map>

namespace mps
{

class JaccardMatch
{
public:
	using LabelT = uint16_t;
	using Match = std::pair<double, boost::bimap<LabelT, LabelT>>;
	using LabelBounds = std::map<LabelT, AABB>;
	Match match;
	JaccardMatch(const cv::Mat& labels1, const cv::Mat& labels2);

	// Intermediate calculations
	LabelBounds boxes1, boxes2;
	Eigen::MatrixXd D;

	boost::bimap<LabelT, int> lblIndex1;
	boost::bimap<LabelT, int> lblIndex2;

	std::map<LabelT, int> iSizes;
	std::map<LabelT, int> jSizes;

	double symmetricCover() const;
};

}

#endif //SRC_JACCARDMATCH_H
