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

#include "mps_voxels/visualization/visualize_voxel_region.h"

namespace mps
{

visualization_msgs::MarkerArray visualize(
	const VoxelRegion& region,
	const VoxelRegion::VertexLabels& labels,
	const std_msgs::Header& header,
	const std::map<VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA>& colormap)
{
	visualization_msgs::Marker m;
	Eigen::Vector3d offset(region.resolution * 0.5, region.resolution * 0.5, region.resolution * 0.5);

	for (size_t i = 0; i < labels.size(); i++)
	{
		if (labels[i] == VoxelRegion::FREE_SPACE) { continue; }
		const VoxelRegion::vertex_descriptor vd = region.vertex_at(i);
		Eigen::Vector3d coord = region.regionMin + region.resolution * (Eigen::Map<const Eigen::Matrix<std::size_t, 3, 1>>(vd.data()).cast<double>()) + offset;

		geometry_msgs::Point cubeCenter;
		cubeCenter.x = coord[0];
		cubeCenter.y = coord[1];
		cubeCenter.z = coord[2];

		m.points.push_back(cubeCenter);

		// Colors
		std_msgs::ColorRGBA color = colormap.at(labels[i]);
		m.colors.push_back(color);
	}

	m.header = header;
	m.ns = "voxel_state";
	m.id = 0;
	m.type = visualization_msgs::Marker::CUBE_LIST;
	m.scale.x = region.resolution;
	m.scale.y = region.resolution;
	m.scale.z = region.resolution;
	m.color.r = 0;
	m.color.g = 0.2;
	m.color.b = 1;
	m.color.a = 1;
	m.pose.orientation.w = 1;

	if (m.points.empty())
		m.action = visualization_msgs::Marker::DELETE;
	else
		m.action = visualization_msgs::Marker::ADD;

	visualization_msgs::MarkerArray vertexLabelVis;
	vertexLabelVis.markers.push_back(m);
	return vertexLabelVis;
}

visualization_msgs::MarkerArray visualize(const VoxelRegion& region,
                                          const VoxelRegion::VertexLabels& labels,
                                          const std_msgs::Header& header,
                                          std::mt19937& re)
{
	assert(region.num_vertices() == labels.size());

	visualization_msgs::Marker m;
	Eigen::Vector3d offset(region.resolution * 0.5, region.resolution * 0.5, region.resolution * 0.5);

	std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));

	std::map<VoxelRegion::VertexLabels::value_type , std_msgs::ColorRGBA> colormap;
	for (size_t i = 0; i < labels.size(); i++)
	{
		if (labels[i] == VoxelRegion::FREE_SPACE) { continue; }
		const VoxelRegion::vertex_descriptor vd = region.vertex_at(i);
		Eigen::Vector3d coord = region.regionMin + region.resolution * (Eigen::Map<const Eigen::Matrix<std::size_t, 3, 1>>(vd.data()).cast<double>()) + offset;

		geometry_msgs::Point cubeCenter;
		cubeCenter.x = coord[0];
		cubeCenter.y = coord[1];
		cubeCenter.z = coord[2];

		m.points.push_back(cubeCenter);

		// Colors
		std_msgs::ColorRGBA color;
		auto iter = colormap.find(labels[i]);
		if (iter != colormap.end())
		{
			color = iter->second;
		}
		else
		{
			// new label
			color.r = uni(re);
			color.g = uni(re);
			color.b = uni(re);
			color.a = 1.0;
			colormap.emplace(labels[i], color);
		}
		m.colors.push_back(color);
	}

	m.header = header;
	m.ns = "voxel_state";
	m.id = 0;
	m.type = visualization_msgs::Marker::CUBE_LIST;
	m.scale.x = region.resolution;
	m.scale.y = region.resolution;
	m.scale.z = region.resolution;
	m.color.r = 0;
	m.color.g = 0.2;
	m.color.b = 1;
	m.color.a = 1;
	m.pose.orientation.w = 1;

	if (m.points.empty())
		m.action = visualization_msgs::Marker::DELETE;
	else
		m.action = visualization_msgs::Marker::ADD;

	visualization_msgs::MarkerArray vertexLabelVis;
	vertexLabelVis.markers.push_back(m);
	return vertexLabelVis;
}

visualization_msgs::MarkerArray visualize(const VoxelRegion& region,
                                          const VoxelRegion::EdgeState& edges,
                                          const std_msgs::Header& header,
                                          std::mt19937& re)
{
	VoxelRegion::VertexLabels vlabels = region.components(const_cast<VoxelRegion::EdgeState&>(edges)).second;

	std::uniform_real_distribution<> uni(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()));
	std::map<int, std_msgs::ColorRGBA> colormap;

	visualization_msgs::Marker m;

	for (VoxelRegion::edges_size_type e = 0; e < region.num_edges(); ++e)
	{
		if (edges[e])
		{
			const VoxelRegion::edge_descriptor edge = region.edge_at(e);
			auto i = region.coordinate_of(source(edge, region));
			auto j = region.coordinate_of(target(edge, region));

			geometry_msgs::Point cubeCenter;
			cubeCenter.x = i[0];
			cubeCenter.y = i[1];
			cubeCenter.z = i[2];

			m.points.push_back(cubeCenter);

			cubeCenter.x = j[0];
			cubeCenter.y = j[1];
			cubeCenter.z = j[2];

			m.points.push_back(cubeCenter);

			// Colors
			std_msgs::ColorRGBA color;
			const auto label = vlabels[region.index_of(source(edge, region))];
			auto iter = colormap.find(label);
			if (iter != colormap.end())
			{
				color = iter->second;
			}
			else
			{
				// new label
				color.r = uni(re);
				color.g = uni(re);
				color.b = uni(re);
				color.a = 1.0;
				colormap.emplace(label, color);
			}

			// Need color for start and end of line
			m.colors.push_back(color);
			m.colors.push_back(color);
		}
	}

	m.header = header;
	m.ns = "edge_state";
	m.id = 0;
	m.type = visualization_msgs::Marker::LINE_LIST;
	m.pose.orientation.w = 1;
	m.scale.x = region.resolution/10.0;
//	m.scale.y = region.resolution/10.0;
//	m.scale.z = region.resolution/10.0;
	m.color.r = 1;
	m.color.g = 0.2;
	m.color.b = 0;
	m.color.a = 1;

	assert(m.points.size() == m.colors.size());

	if (m.points.empty())
		m.action = visualization_msgs::Marker::DELETE;
	else
		m.action = visualization_msgs::Marker::ADD;

	visualization_msgs::MarkerArray vertexLabelVis;
	vertexLabelVis.markers.push_back(m);
	return vertexLabelVis;
}

}
