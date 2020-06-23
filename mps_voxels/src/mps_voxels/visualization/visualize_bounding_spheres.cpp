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

#include "mps_voxels/visualization/visualize_bounding_spheres.h"

namespace mps
{

visualization_msgs::MarkerArray visualize(const std::map<std::string, std::shared_ptr<MotionModel>>& data, const std_msgs::Header& header, std::mt19937&)
{
	visualization_msgs::MarkerArray markers;
	int id = 0;
	for (const auto& model : data)
	{
		visualization_msgs::Marker m;
		m.ns = "collision";
		m.id = id++;
		m.type = visualization_msgs::Marker::SPHERE;
		m.action = visualization_msgs::Marker::ADD;
		m.scale.x = m.scale.y = m.scale.z = model.second->boundingSphere.radius*2.0;
		m.color.a = 0.5f;
		Eigen::Vector3d p = model.second->localTglobal.inverse() * model.second->boundingSphere.center;
		m.pose.position.x = p.x();
		m.pose.position.y = p.y();
		m.pose.position.z = p.z();
		m.pose.orientation.w = 1.0;
		m.frame_locked = true;
		m.header = header;
		markers.markers.push_back(m);
	}
	return markers;
}

}
