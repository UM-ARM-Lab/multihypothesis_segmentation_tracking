//
// Created by kunhuang on 2/12/20.
//

#include "mps_voxels/Particle.h"
#include "mps_voxels/OccupancyData.h"
#include "mps_voxels/moveit_pose_type.h"
#include "mps_voxels/project_point.hpp"

#include <image_geometry/pinhole_camera_model.h>

namespace mps
{

std::set<ObjectIndex> getUniqueObjectLabels(const VoxelRegion::VertexLabels& input)
{
	std::set<ObjectIndex> out;

#pragma omp declare reduction (merge : std::set<ObjectIndex> : omp_out.insert(omp_in.begin(), omp_in.end()))
#pragma omp parallel for reduction(merge: out)
	for (size_t i = 0; i < input.size(); ++i)
	{
		out.insert(ObjectIndex(input[i]));
	}
	out.erase(ObjectIndex());
	return out;
}

cv::Mat rayCastParticle(const Particle& particle, const image_geometry::PinholeCameraModel& cameraModel, const moveit::Pose& worldTcamera, const int& step)
{
	auto roi = cameraModel.rectifiedRoi();

	using LabelT = uint8_t;
	using DepthT = float;
	cv::Mat labels = cv::Mat::zeros(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_8U);
	cv::Mat depthBuf(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_32F, std::numeric_limits<DepthT>::max());

	std::map<ObjectIndex, std::shared_ptr<octomap::OcTree>> labelToOcTreeLookup = particle.state->voxelRegion->vertexLabelToOctrees(particle.state->vertexState, particle.state->uniqueObjectLabels);
	std::cerr << "labelToOcTreeLookup contains " << labelToOcTreeLookup.size() << " elements." << std::endl;

	const Eigen::Vector3d r0_world = worldTcamera.translation();
	const octomap::point3d cameraPose(r0_world.x(), r0_world.y(), r0_world.z());

	roi.y = 0; roi.height = cameraModel.cameraInfo().height;
	roi.x = 0; roi.width = cameraModel.cameraInfo().width;

	//TODO: TOO SLOW!!!
#pragma omp parallel for
	for (int v = roi.y; v < roi.y+roi.height; /*++v*/v+=step)
	{
		for (int u = roi.x; u < roi.x + roi.width; /*++u*/u+=step)
		{
			const Eigen::Vector3d rn_world = worldTcamera.linear() * toPoint3D<Eigen::Vector3d>(u, v, 0.5, cameraModel).normalized();
			const octomap::point3d rayPoint(rn_world.x(), rn_world.y(), rn_world.z());

			for (auto& pair : labelToOcTreeLookup)
			{
				octomap::point3d intsectPoint;
				if (pair.second->castRay(cameraPose, rayPoint, intsectPoint, true, 2.0))
				{
//					std::cerr << "Intersection with octree " << pair.first << std::endl;
					double dist = (intsectPoint - cameraPose).norm();
					auto& zBuf = depthBuf.at<DepthT>(v, u);
					if (dist < zBuf)
					{
						zBuf = dist;
						labels.at<LabelT>(v, u) = pair.first.id;
//						std::cerr << "label value: " << labels.at<LabelT>(v, u) << std::endl;
					}
				}
			}
		}
	}

	return labels;
}

}