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
#include "mps_voxels/visualization/visualize_object.h"
#include "mps_voxels/octree_utils.h"
#include "mps_voxels/util/containers.hpp"

#include <geometric_shapes/shape_operations.h>

namespace mps
{

visualization_msgs::MarkerArray visualize(const Object& obj, const std_msgs::Header& header, const std_msgs::ColorRGBA& color)
{
	const auto& subtree = obj.occupancy;

	visualization_msgs::MarkerArray ma = visualizeOctree(subtree.get(), header.frame_id, &color);
	const std::string name = "completed_"+std::to_string(std::abs(obj.index.id));
	for (auto& m : ma.markers)
	{
		m.ns = name;
		m.header.stamp = header.stamp;
	}

	// Visualize approximate shape
	visualization_msgs::Marker m;
	const auto& approx = obj.approximation;
	shapes::constructMarkerFromShape(approx.get(), m, true);
	m.id = std::abs(obj.index.id);
	m.ns = "bounds";
	m.header = header;
	m.pose.orientation.w = 1;
	m.color = color; m.color.a = 0.6;
	m.frame_locked = true;
	visualization_msgs::MarkerArray ms;
	ms.markers.push_back(m);

	ma.markers += ms.markers;

	return ma;
}

visualization_msgs::MarkerArray visualize(const Object& obj, const std_msgs::Header& header, std::default_random_engine& re)
{
	std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));

	std_msgs::ColorRGBA color;
	color.r = uni(re);
	color.g = uni(re);
	color.b = uni(re);
	color.a = 1.0;

	return visualize(obj, header, color);
}

}
