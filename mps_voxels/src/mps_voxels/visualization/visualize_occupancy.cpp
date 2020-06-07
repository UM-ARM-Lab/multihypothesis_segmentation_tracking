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

#include "mps_voxels/visualization/visualize_occupancy.h"
#include "mps_voxels/visualization/visualize_voxel_region.h"
#include "mps_voxels/visualization/visualize_object.h"
#include "mps_voxels/util/containers.hpp"

namespace mps
{

visualization_msgs::MarkerArray visualize(
	const OccupancyData& data,
	const std_msgs::Header& header,
	const std::map<VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>& colormap)
{
	visualization_msgs::MarkerArray ma = visualize(*data.voxelRegion, data.vertexState, header, colormap);
//	ma.markers += visualize(*data.voxelRegion, data.edgeState, header, re).markers;
	for (const auto& pair : data.objects)
	{
		ma.markers += visualize(*pair.second, header, colormap.at(pair.first.id)).markers;
	}

	return ma;
}

visualization_msgs::MarkerArray visualize(const OccupancyData& data, const std_msgs::Header& header, std::default_random_engine& re)
{
	visualization_msgs::MarkerArray ma = visualize(*data.voxelRegion, data.vertexState, header, re);
	ma.markers += visualize(*data.voxelRegion, data.edgeState, header, re).markers;
	for (const auto& pair : data.objects)
	{
		ma.markers += visualize(*pair.second, header, re).markers;
	}

	return ma;
}

}