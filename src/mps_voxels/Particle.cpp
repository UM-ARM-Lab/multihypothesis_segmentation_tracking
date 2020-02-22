//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/Particle.h"
#include "mps_voxels/OccupancyData.h"
#include "mps_voxels/moveit_pose_type.h"
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

cv::Mat rayCastParticle(const Particle& particle, const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera, const int& step)
{
	auto roi = cameraModel.rectifiedRoi();

	using LabelT = uint16_t;
	using DepthT = float;
	cv::Mat labels = cv::Mat::zeros(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_16U);
	cv::Mat depthBuf(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_32F, std::numeric_limits<DepthT>::max());

//	std::map<ObjectIndex, std::shared_ptr<octomap::OcTree>> labelToOcTreeLookup = particle.state->voxelRegion->vertexLabelToOctrees(particle.state->vertexState, particle.state->uniqueObjectLabels);
//	std::cerr << "labelToOcTreeLookup contains " << labelToOcTreeLookup.size() << " elements." << std::endl;

	const Eigen::Vector3f r0_world = worldTcamera.translation().cast<float>();
	const octomap::point3d cameraPose(r0_world.x(), r0_world.y(), r0_world.z());

	roi.y = 0; roi.height = cameraModel.cameraInfo().height;
	roi.x = 0; roi.width = cameraModel.cameraInfo().width;

	// TODO: TOO SLOW!!!
#pragma omp parallel for
	for (int v = roi.y; v < roi.y+roi.height; /*++v*/v+=step)
	{
		for (int u = roi.x; u < roi.x + roi.width; /*++u*/u+=step)
		{
			const Eigen::Vector3f rn_world = (worldTcamera.linear() * toPoint3D<Eigen::Vector3d>(u, v, 0.5, cameraModel).normalized()).cast<float>();
			const octomap::point3d rayPoint(rn_world.x(), rn_world.y(), rn_world.z());

			const Eigen::Vector3f rnInv_world = rn_world.cast<float>().cwiseInverse();

			for (const auto& pair : particle.state->objects)
			{
				// Check AABB intersection
				const auto& obj = pair.second; // particle.state->objects.at(pair.first);
				bool mightHit = fast_ray_box_intersection(r0_world, rnInv_world, obj->minExtent, obj->maxExtent);
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
						labels.at<LabelT>(v, u) = pair.first.id + 1; //// voxel label == -1: free sapce; seg label == 0: free space;
//						std::cerr << "label value: " << labels.at<LabelT>(v, u) << std::endl;
					}
				}
			}
		}
	}
	return labels;
}

void refineParticleFreeSpace(Particle& particle)
{

}

}