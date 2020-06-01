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

#include "mps_voxels/OccupancyData.h"
#include "mps_voxels/moveit_pose_type.h"
#include "mps_voxels/Scene.h"

#include "mps_voxels/LocalOctreeServer.h" // for setBBox()
#include "mps_voxels/shape_utils.h"
#include "mps_voxels/project_point.hpp"

#include <image_geometry/pinhole_camera_model.h>

// From https://tavianator.com/cgit/dimension.git/tree/libdimension/bvh/bvh.c#n196
// NB: Will miss if ray is coplanar with a box face
bool
fast_ray_box_intersection(const Eigen::Vector3f& rayOrigin,
                          const Eigen::Vector3f& rayVectorInv,
                          const Eigen::Vector3f& bboxMin,
                          const Eigen::Vector3f& bboxMax)
{
// This is actually correct, even though it appears not to handle edge cases
// (ray.n.{x,y,z} == 0).  It works because the infinities that result from
// dividing by zero will still behave correctly in the comparisons.  Rays
// which are parallel to an axis and outside the box will have tmin == inf
// or tmax == -inf, while rays inside the box will have tmin and tmax
// unchanged.

	double tx1 = (bboxMin.x() - rayOrigin.x())*rayVectorInv.x();
	double tx2 = (bboxMax.x() - rayOrigin.x())*rayVectorInv.x();

	double tmin = std::min(tx1, tx2);
	double tmax = std::max(tx1, tx2);

	double ty1 = (bboxMin.y() - rayOrigin.y())*rayVectorInv.y();
	double ty2 = (bboxMax.y() - rayOrigin.y())*rayVectorInv.y();

	tmin = std::max(tmin, std::min(ty1, ty2));
	tmax = std::min(tmax, std::max(ty1, ty2));

	double tz1 = (bboxMin.z() - rayOrigin.z())*rayVectorInv.z();
	double tz2 = (bboxMax.z() - rayOrigin.z())*rayVectorInv.z();

	tmin = std::max(tmin, std::min(tz1, tz2));
	tmax = std::min(tmax, std::max(tz1, tz2));

	return tmax >= std::max(0.0, tmin);
}

namespace mps
{

OccupancyData::OccupancyData(std::shared_ptr<VoxelRegion> region)
	: voxelRegion(std::move(region)),
	  vertexState(voxelRegion->num_vertices(), VoxelRegion::FREE_SPACE),
	  edgeState(voxelRegion->num_edges(), false)
{

}

OccupancyData::OccupancyData(const std::shared_ptr<const Scene>& scene,
                             std::shared_ptr<VoxelRegion> region,
                             VoxelRegion::VertexLabels state)
	: voxelRegion(std::move(region)),
	  vertexState(std::move(state)),
	  edgeState(voxelRegion->num_edges(), false),
	  uniqueObjectLabels(getUniqueObjectLabels(vertexState))
{
	// There's currently ~1.5M edges. Reserve this for a sparser state representation
	// Build the edge representation
//	for (VoxelRegion::edges_size_type e = 0; e < voxelRegion->num_edges(); ++e)
//	{
//		const VoxelRegion::edge_descriptor edge = voxelRegion->edge_at(e);
//		auto i = voxelRegion->index_of(source(edge, voxelRegion));
//		auto j = voxelRegion->index_of(target(edge, voxelRegion));
//
//		edgeState[e] = (vertexState[i] == vertexState[j]);
//	}

	// Allocate the object representation
	for (const auto& id : uniqueObjectLabels)
	{
		std::shared_ptr<octomap::OcTree> subtree(scene->sceneOctree->create());
		subtree->setProbMiss(0.05);
		subtree->setProbHit(0.95);
		setBBox(scene->minExtent, scene->maxExtent, subtree.get());
		auto res = objects.emplace(id, std::make_unique<Object>(id, subtree));
		res.first->second->minExtent = Eigen::Vector3d::Constant(std::numeric_limits<double>::max());
		res.first->second->maxExtent = Eigen::Vector3d::Constant(std::numeric_limits<double>::lowest());
	}

	// Populate the object representation
	for (mps::VoxelRegion::vertices_size_type v = 0; v < voxelRegion->num_vertices(); ++v)
	{
		const auto label = vertexState[v];
		if (label == VoxelRegion::FREE_SPACE) { continue; }
		const auto descriptor = voxelRegion->vertex_at(v);
		const auto coordinate = voxelRegion->coordinate_of(descriptor);

		ObjectIndex objID(label);
		auto& obj = objects.at(objID);
		obj->occupancy->setNodeValue(coordinate.x(), coordinate.y(), coordinate.z(),
		                             std::numeric_limits<float>::max(), true);
		obj->minExtent = obj->minExtent.cwiseMin(coordinate);
		obj->maxExtent = obj->maxExtent.cwiseMax(coordinate);
	}

	// Prune and approximate shape data
	for (auto& pair : objects)
	{
		pair.second->occupancy->updateInnerOccupancy();
		pair.second->approximation = approximateShape(pair.second->occupancy.get());
	}

	segInfo = std::make_shared<SegmentationInfo>();
	*segInfo = *scene->segInfo; // Shallow copy; deep copy objectness later

	const auto& os = scene->segInfo->objectness_segmentation;
	segInfo->objectness_segmentation = boost::make_shared<cv_bridge::CvImage>(
		os->header, os->encoding,
		cv::Mat(os->image.size(),
		        os->image.type()));

	const int segStep = 1;
	segInfo->objectness_segmentation->image = rayCastOccupancy(*this, scene->cameraModel, scene->worldTcamera, scene->roi, segStep);

	if (scene->scenario->shouldVisualize("particle_segmentation"))
	{
//		IMSHOW()
	}
}


cv::Mat rayCastOccupancy(const OccupancyData& state, const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera, const cv::Rect& roi, const int& step)
{
//	auto roi = cameraModel.rectifiedRoi();

	using LabelT = uint16_t;
	using DepthT = float;
	cv::Mat labels = cv::Mat::zeros(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_16U);
	cv::Mat depthBuf(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_32F, std::numeric_limits<DepthT>::max());

//	std::map<ObjectIndex, std::shared_ptr<octomap::OcTree>> labelToOcTreeLookup = particle.state->voxelRegion->vertexLabelToOctrees(particle.state->vertexState, particle.state->uniqueObjectLabels);
//	std::cerr << "labelToOcTreeLookup contains " << labelToOcTreeLookup.size() << " elements." << std::endl;

	const Eigen::Vector3f r0_world = worldTcamera.translation().cast<float>();
	const octomap::point3d cameraPose(r0_world.x(), r0_world.y(), r0_world.z());

//	roi.y = 0; roi.height = cameraModel.cameraInfo().height;
//	roi.x = 0; roi.width = cameraModel.cameraInfo().width;

	// TODO: TOO SLOW!!!
#pragma omp parallel for
	for (int v = roi.y; v < roi.y+roi.height; /*++v*/v+=step)
	{
		for (int u = roi.x; u < roi.x + roi.width; /*++u*/u+=step)
		{
			const Eigen::Vector3f rn_world = (worldTcamera.linear() * toPoint3D<Eigen::Vector3d>(u, v, 0.5, cameraModel).normalized()).cast<float>();
			const octomap::point3d rayPoint(rn_world.x(), rn_world.y(), rn_world.z());

			const Eigen::Vector3f rnInv_world = rn_world.cast<float>().cwiseInverse();

			for (const auto& pair : state.objects)
			{
				// Check AABB intersection
				const auto& obj = pair.second; // particle.state->objects.at(pair.first);
				bool mightHit = fast_ray_box_intersection(r0_world, rnInv_world, obj->minExtent.cast<float>(), obj->maxExtent.cast<float>());
				if (!mightHit) { continue; }

				octomap::point3d intsectPoint;
				if (obj->occupancy->castRay(cameraPose, rayPoint, intsectPoint, true, 2.0))
				{
//					std::cerr << "Intersection with octree " << pair.first << std::endl;
					double dist = (intsectPoint - cameraPose).norm();
					auto& zBuf = depthBuf.at<DepthT>(v, u);
					if (dist < zBuf)
					{
						zBuf = dist;
						labels.at<LabelT>(v, u) = pair.first.id; //// voxel label == 0: free sapce; seg label == 0: free space;
//						std::cerr << "label value: " << labels.at<LabelT>(v, u) << std::endl;
					}
				}
			}
		}
	}
	return labels;
}

}
