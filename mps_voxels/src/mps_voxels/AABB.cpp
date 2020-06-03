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

#include "mps_voxels/AABB.h"
#include "mps_voxels/image_utils.h"

#include <mutex>
#include <utility>

namespace mps
{

class ParallelBBox : public cv::ParallelLoopBody
{
public:

	ParallelBBox(cv::Mat inputImage, std::map<uint16_t, AABB>& presult)
		: m_img(std::move(inputImage)), result(presult)
	{
		// Pre-allocate storage for AABBs and their mutexes
		for (const auto l : unique(m_img))
		{
			mtxs[l];
			result[l];
		}
	}

	void operator ()(const cv::Range& range) const CV_OVERRIDE
	{
		for (int r = range.start; r < range.end; r++)
		{
			// Compute image coordinates
			int v = r / m_img.cols;
			int u = r % m_img.cols;

			// Look up label for this pixel
			auto label = m_img.ptr<uint16_t>(v)[u];

			// Lock and grow the associated AABB
			std::lock_guard<std::mutex> lock(mtxs.at(label));
			result.at(label).grow(u, v);
		}
	}

protected:
	const cv::Mat m_img;
	std::map<uint16_t, AABB>& result;
	mutable std::map<uint16_t, std::mutex> mtxs;
};

std::map<uint16_t, AABB> getBBoxes(const cv::Mat& labels)
{
	std::map<uint16_t, AABB> res;
	ParallelBBox parallelBBox(labels, res);
	parallel_for_(cv::Range(0, labels.rows*labels.cols), parallelBBox);
	return res;
}

}
