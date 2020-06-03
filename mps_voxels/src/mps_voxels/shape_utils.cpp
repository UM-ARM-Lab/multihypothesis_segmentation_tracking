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

#include "mps_voxels/shape_utils.h"
#include "mps_voxels/shape_utils_impl.hpp"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/util/assert.h"

#include <octomap/octomap_types.h>

#include <geometric_shapes/shape_operations.h>

namespace mps
{

template std::shared_ptr<shapes::Mesh> convex_hull<octomap::point3d_collection, octomap::point3d>(const octomap::point3d_collection&);
template std::shared_ptr<shapes::Mesh> prism<octomap::point3d_collection, octomap::point3d>(const octomap::point3d_collection&);
template std::shared_ptr<shapes::Mesh> ZAMBB<octomap::point3d_collection, octomap::point3d>(const octomap::point3d_collection&);

std::shared_ptr<shapes::Mesh> approximateShape(const octomap::OcTree* tree)
{
	// Convex hull
	octomap::point3d_collection pts = getPoints(tree);
//	std::shared_ptr<shapes::Mesh> hull = convex_hull(pts);
	std::shared_ptr<shapes::Mesh> hull = prism(pts);
//	std::shared_ptr<shapes::Mesh> hull = ZAMBB(pts);
	return hull;
}

void getAABB(const shapes::Mesh& shape, Eigen::Vector3d& min, Eigen::Vector3d& max)
{
	min = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
	max = Eigen::Vector3d::Constant(-std::numeric_limits<double>::infinity());
//	const int DIM = 3;
//	for (int d=0; d < DIM; ++d)
//	{
//		min[d] = std::numeric_limits<float>::infinity();
//		max[d] = -std::numeric_limits<float>::infinity();
//	}

//	#pragma omp parallel for reduction(max : max) reduction(min : min)
	for (unsigned i = 0; i < shape.vertex_count; ++i)
	{
		Eigen::Map<Eigen::Vector3d> vertex(shape.vertices + (3*i));
		min = min.cwiseMin(vertex);
		max = max.cwiseMax(vertex);
//		for (int d=0; d < DIM; ++d)
//		{
//			min[d] = std::min(min[d], static_cast<float>(vertex[d]));
//			max[d] = std::max(max[d], static_cast<float>(vertex[d]));
//		}
	}
}

}
