/**
 * \file geometry.cpp
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

#include "mps_voxels/util/geometry.h"

namespace mps
{

// WRF pnpoly modified for Eigen
// http://www.ecse.rpi.edu/Homepages/wrf/Research/Short_Notes/pnpoly.html
bool pnpoly(const SupportPolygon& verts, const Eigen::Vector2d& test)
{
	const int nvert = verts.size();
	int i, j;
	bool c = false;
	for (i = 0, j = nvert-1; i < nvert; j = i++)
	{
		if ( ((verts[i].y()>test.y()) != (verts[j].y()>test.y())) &&
		     (test.x() < (verts[j].x()-verts[i].x()) * (test.y()-verts[i].y()) / (verts[j].y()-verts[i].y()) + verts[i].x()) )
			c = !c;
	}
	return c;
}

}
