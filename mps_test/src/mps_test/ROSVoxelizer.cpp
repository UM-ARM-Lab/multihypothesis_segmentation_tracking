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

#include "mps_test/ROSVoxelizer.h"
#include "mps_simulation/GazeboModel.h"

mps::ROSVoxelizer::ROSVoxelizer(std::vector<std::shared_ptr<GazeboModel>> _models)
	: models(std::move(_models))
{

}

mps::VoxelRegion::VertexLabels
mps::ROSVoxelizer::voxelize(const mps::VoxelRegion& region, const std::vector<Eigen::Isometry3d>& poses)
{
	mps::VoxelRegion::VertexLabels labels(region.num_vertices(), mps::VoxelRegion::FREE_SPACE);

	#pragma omp parallel for
	for (size_t i = 0; i < region.m_num_vertices; ++i)
	{
		const Eigen::Vector3d X_world = region.coordinate_of(region.vertex_at(i));

		for (size_t m = 0; m < models.size(); ++m)
		{
			const GazeboModel& model = *models[m];

			const Eigen::Isometry3d& worldTshape = poses[m];
			Eigen::Isometry3d shapeTworld = worldTshape.inverse(Eigen::Isometry);

			const Eigen::Vector3d X_model = shapeTworld * X_world;

			if (mps::pointIntersectsModel(X_model, model))
			{
				labels[i] = m + 1;
				break;
			}
		}
	}

	return labels;
}
