/**
 * \file geometry.h
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

#ifndef MPS_GEOMETRY_H
#define MPS_GEOMETRY_H

#include <Eigen/Geometry>
#include <Eigen/StdVector>

namespace mps
{

using SupportPolygon = std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d>>;

bool pnpoly(const SupportPolygon& verts, const Eigen::Vector2d& test);

/**
 * @brief rayLineSegmentIntersection
 * @param[in] o Ray origin
 * @param[in] v Ray vector
 * @param[in] a Segment endpoint a
 * @param[in] b Segment endpoint b
 * @param[out] x intersection
 * @return true if intersects
 */
template<typename Scalar>
bool rayLineSegmentIntersection(const Eigen::Matrix<Scalar,2,1>& o, const Eigen::Matrix<Scalar,2,1>& v,
                                const Eigen::Matrix<Scalar,2,1>& a, const Eigen::Matrix<Scalar,2,1>& b,
                                Eigen::Matrix<Scalar,2,1>& x)
{
	Eigen::Matrix<Scalar,2,1> u((b.x() - a.x()), (b.y() - a.y()));
	x = Eigen::Matrix<Scalar,2,1>(NAN, NAN);
	float r, s, d;
	// Make sure the lines aren't parallel
	if (u.x()*v.y() != u.y()*v.x())//(v.y() / v.x() != u.y() / u.x())
	{
		d = ((v.x() * u.y()) - v.y() * u.x());
		if (d != 0)
		{
			r = (((o.y() - a.y()) * u.x()) - (o.x() - a.x()) * u.y()) / d;
			s = (((o.y() - a.y()) * v.x()) - (o.x() - a.x()) * v.y()) / d;
			if (r >= 0)
			{
				if (s >= 0 && s <= 1)
				{
					x = o + (r * v);
					return true;
				}
			}
		}
	}
	return false;
}

}

#endif // MPS_GEOMETRY_H
