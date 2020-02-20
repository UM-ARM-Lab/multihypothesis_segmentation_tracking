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

#include "mps_voxels/SegmentationTreeSampler.h"
#include "mps_voxels/Ultrametric.h"
#include "mps_voxels/ValueTree.h"
#include "mps_voxels/ValueTree_impl.hpp"
#include "mps_voxels/SceneCut.h"
#include "mps_voxels/MCMCTreeCut.h"
#include "mps_voxels/MCMCTreeCut_impl.hpp"
#include "mps_voxels/relabel_tree_image.hpp"

// Still needed for cv_bridge currently
#include <boost/make_shared.hpp>

namespace mps
{


SegmentationTreeSampler::SegmentationTreeSampler(std::shared_ptr<const SegmentationInfo> original_, const Options& opts) // NOLINT(cppcoreguidelines-pro-type-member-init,hicpp-member-init)
	: Belief<SegmentationCut>(),
	  originalInfo(std::move(original_))
{
	um = std::make_unique<Ultrametric>(originalInfo->ucm2, originalInfo->labels2);
	sc = std::make_unique<SceneCut>();
	vt = sc->process(*um, originalInfo->labels);

	std::tie(vStar, cutStar) = optimalCut(vt);

	samplingSigmaSquared = opts.sigmaGain * vStar * vStar;

	mcmc = std::make_unique<tree::MCMCTreeCut<tree::DenseValueTree>>(vt, samplingSigmaSquared);
	mcmc->nTrials = opts.autoCorrelationSteps;
}

std::pair<double, SegmentationCut>
SegmentationTreeSampler::sample(RNG& rng, const SAMPLE_TYPE type)
{
	auto res = mcmc->sample(rng, type);

	const auto& os = originalInfo->objectness_segmentation;
	SegmentationInfo newSeg = *originalInfo; // Mostly shallow copy
	newSeg.objectness_segmentation = boost::make_shared<cv_bridge::CvImage>(
		os->header, os->encoding,
		cv::Mat(os->image.size(),
		        os->image.type()));

	std::map<int, int> index_to_label;
	for (const auto& pair : um->label_to_index)
	{
		index_to_label.insert({pair.second, pair.first});
	}

	newSeg.objectness_segmentation->image =
		relabelCut(vt, res.second, index_to_label, newSeg.labels);


	return {res.first, {newSeg, res.second}};
}

SegmentationTreeSampler::~SegmentationTreeSampler() = default;

}
