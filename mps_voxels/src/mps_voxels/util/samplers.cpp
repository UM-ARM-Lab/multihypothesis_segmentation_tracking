/**
 * \file samplers.cpp
 * \brief
 *
 * \author Andrew Price
 * \date 2016-7-8
 *
 * \copyright
 *
 * Copyright (c) 2020, Andrew Price
 * All rights reserved.
 *
 * This file is provided under the following "BSD-style" License:
 * Redistribution and use in source and binary forms, with or
 * without modification, are permitted provided that the following
 * conditions are met:
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above
 *   copyright notice, this list of conditions and the following
 *   disclaimer in the documentation and/or other materials provided
 *   with the distribution.
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 * USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "mps_voxels/util/samplers.h"
#include "mps_voxels/util/geometry.h"

namespace mps
{

SimplexSampler::SimplexSampler(const size_t n) : N(n)
{
	rng = std::mt19937(rd());
	generator = std::uniform_real_distribution<double>(0, 1);
}

Eigen::VectorXd SimplexSampler::getPoint()
{
	Eigen::VectorXd pt(N);
	for (size_t i = 0; i < N; ++i)
	{
		pt[i] = -log(1.0-generator(rng)); // exponentially distributed samples, generator -> [0,1)
	}
	pt /= pt.sum(); // normalize the sum
	return pt;
}

UniformPolygonRejectionSampler::UniformPolygonRejectionSampler(const SupportPolygon& p)
    : mPoly(p),
      mSimplexSampler(p.size()),
      mR(&(mPoly.front()[0]), 2, mPoly.size())
{
	rng = std::mt19937(rd());

	Eigen::AlignedBox2d aabb(p.front());
	for (const Eigen::Vector2d& pt : p)
	{
		aabb.extend(pt);
	}

	for (size_t i = 0; i < 2; ++i)
	{
		generators[i] = std::uniform_real_distribution<double>(aabb.min()[i], aabb.max()[i]);
	}
}

Eigen::Vector2d UniformPolygonRejectionSampler::getPoint()
{
	Eigen::Vector2d pt;
	for (size_t attempt = 0; attempt < 20; ++attempt)
	{
		for (size_t i = 0; i < 2; ++i)
		{
			pt[i] = generators[i](rng);
		}
		if (pnpoly(mPoly, pt))
		{
			return pt;
		}
	}

	// Failed to sample inside polygon, use convex combination of points instead
	return mR * mSimplexSampler.getPoint();
}

}
