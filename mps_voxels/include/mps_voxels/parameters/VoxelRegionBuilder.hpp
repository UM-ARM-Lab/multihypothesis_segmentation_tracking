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

#ifndef MPS_VOXELREGIONBUILDER_HPP
#define MPS_VOXELREGIONBUILDER_HPP

#include "mps_voxels/parameters/ParameterBlock.hpp"
#include "mps_voxels/VoxelRegion.h"

namespace mps
{

struct VoxelRegionBuilder
{
	template <typename ParameterProvider>
	static VoxelRegion build(const ParameterProvider& p)
	{
		Eigen::Vector3d minExtent(getParam<double>(p, "min/x"),
		                          getParam<double>(p, "min/y"),
		                          getParam<double>(p, "min/z"));
		Eigen::Vector3d maxExtent(getParam<double>(p, "max/x"),
		                          getParam<double>(p, "max/y"),
		                          getParam<double>(p, "max/z"));
		double resolution = getParam<double>(p, "resolution");

		return {resolution, minExtent, maxExtent};
	}
};

}
#endif //MPS_VOXELREGIONBUILDER_HPP
