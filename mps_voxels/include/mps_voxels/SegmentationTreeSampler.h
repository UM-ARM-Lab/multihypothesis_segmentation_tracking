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

#ifndef MPS_SEGMENTATIONTREESAMPLER_H
#define MPS_SEGMENTATIONTREESAMPLER_H

#include "mps_voxels/Belief.hpp"
#include "mps_voxels/SegmentationInfo.h"

#include "mps_voxels/ValueTree.h"

struct Ultrametric;

namespace mps
{

class SceneCut;

namespace tree
{
template <typename T>
class MCMCTreeCut;
}

struct SegmentationCut
{
	SegmentationInfo segmentation;
	tree::TreeCut cut;
};

class SegmentationTreeSampler : public Belief<SegmentationCut>
{
public:
	std::shared_ptr<const SegmentationInfo> originalInfo;

	std::unique_ptr<Ultrametric> um;
	std::unique_ptr<SceneCut> sc;
	tree::DenseValueTree vt;
	tree::TreeCut cutStar; ///< Optimal Cut
	double vStar; ///< Value of Optimal Cut
	double samplingSigmaSquared;

	struct Options
	{
		explicit Options() {} // NOLINT(hicpp-use-equals-default,modernize-use-equals-default) (Clang/GCC Bug)
		double sigmaGain = 0.25; ///< Scaling factor for sampling sigma
		int autoCorrelationSteps = 20; ///< Mixing number to reduce MCMC autocorrelation
	};

	std::unique_ptr<tree::MCMCTreeCut<tree::DenseValueTree>> mcmc;

	explicit
	SegmentationTreeSampler(std::shared_ptr<const SegmentationInfo> originalInfo, const Options& opts = Options());
	~SegmentationTreeSampler();

	std::pair<double, SegmentationCut> sample(RNG& rng, const SAMPLE_TYPE type) override;
	std::vector<std::pair<double, SegmentationCut>> sample(RNG& rng, const size_t nSamples, const bool maximumFirst);
};

}

#endif // MPS_SEGMENTATIONTREESAMPLER_H
