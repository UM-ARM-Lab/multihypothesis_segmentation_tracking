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

#ifndef MPS_SHAPE_UTILS_H
#define MPS_SHAPE_UTILS_H

//#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>
#include <octomap/octomap_types.h>
#include <Eigen/Core>

namespace mps
{

template <typename PointContainerT, typename PointT = typename PointContainerT::value_type>
std::shared_ptr<shapes::Mesh> convex_hull(const PointContainerT& points);

template <typename PointContainerT, typename PointT = typename PointContainerT::value_type>
std::shared_ptr<shapes::Mesh> prism(const PointContainerT& points);

/**
 * @brief ZAMBB Compute the Z-Aligned Minimum-volume Bounding Box
 * @tparam PointContainerT A container of 3D points
 * @tparam PointT The type of
 * @param points
 * @return
 */
template <typename PointContainerT, typename PointT = typename PointContainerT::value_type>
std::shared_ptr<shapes::Mesh> ZAMBB(const PointContainerT& points);

std::shared_ptr<shapes::Mesh> approximateShape(const octomap::OcTree* tree);

void getAABB(const shapes::Mesh& shape, Eigen::Vector3d& min, Eigen::Vector3d& max);

}

#endif // MPS_SHAPE_UTILS_H
