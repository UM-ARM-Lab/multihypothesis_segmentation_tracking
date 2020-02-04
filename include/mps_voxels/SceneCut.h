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

#ifndef SRC_SCENECUT_H
#define SRC_SCENECUT_H

#include "mps_voxels/Ultrametric.h"
#include "mps_voxels/ValueTree.h"

namespace mps
{

//function scores  = compute_objectness_scores(features)
//
//start_ths = features.start_ths';
//end_ths = features.end_ths';
//
//sigma_i = 0.3;
//sigma_o = 0.7;
//prior_prob = 0.5;
//region_scores = prior_prob.*exp(-abs(start_ths - 0).^2./sigma_i^2).*exp(-abs(end_ths - 1).^2./sigma_o^2);
//Areas = features.areas;
//too_big_small_regions_ids = Areas>150000 | Areas < 1000;
//region_scores(too_big_small_regions_ids) = 0.001;
//scores = log(region_scores).*(Areas.^0.95);
//
//end

class SceneCut
{
public:
	// Accumulate magic numbers from SceneCut paper
	struct Parameters
	{
		double sigma_i = 0.3;
		double sigma_o = 0.7;
		double prior_prob = 0.5;
		int max_area = 150000;
		int min_area = 1000;
		double out_of_area_range_weight = 0.001;
		double area_exponent = 0.95;
	};

	Parameters parameters;

	ValueTree process(const Ultrametric& um, const cv::Mat& labels) const;
};

}

#endif // SRC_SCENECUT_H
