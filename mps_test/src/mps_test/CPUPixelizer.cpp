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

#include "mps_test/CPUPixelizer.h"
#include <mps_voxels/project_point.hpp>

#include <opencv2/imgproc.hpp>

mps::CPUPixelizer::CPUPixelizer(std::vector<std::shared_ptr<GazeboModel>> _models)
	: ScenePixelizer(), models(std::move(_models))
{ }

cv::Mat mps::CPUPixelizer::pixelize(const image_geometry::PinholeCameraModel& cameraModel,
                                    const Eigen::Isometry3d& worldTcamera, const std::vector<GazeboModelState>& states)
{
	const auto roi = cameraModel.rectifiedRoi();

	using LabelT = uint16_t;
	using DepthT = double;
	cv::Mat labels = cv::Mat::zeros(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_16U);
	depthBuf = cv::Mat(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_64F,
	                   std::numeric_limits<DepthT>::max());

	const int step = 1;

	#pragma omp parallel for
	for (int v = roi.y; v < roi.y + roi.height; /*++v*/v += step)
	{
		for (int u = roi.x; u < roi.x + roi.width; /*++u*/u += step)
		{
			const Eigen::Vector3d rn_world = worldTcamera.linear()
			                                 * toPoint3D<Eigen::Vector3d>(u, v, 1.0, cameraModel).normalized();
			const Eigen::Vector3d r0_world = worldTcamera.translation();

//				tf::Transform temp;
//				Eigen::Isometry3d worldTray = Eigen::Isometry3d::Identity(); worldTray.translation() = r0_world + rn_world;
//				tf::poseEigenToTF(worldTray, temp);
//				broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), worldFrame, "ray"));
//				ros::spinOnce(); ros::Duration(0.1).sleep();

//				Eigen::Isometry3d cameraTray = Eigen::Isometry3d::Identity(); cameraTray.translation() = toPoint3D<Eigen::Vector3d>(u, v, 1.0, cameraModel).normalized();
//				tf::poseEigenToTF(cameraTray, temp);
//				broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), cameraFrame, "ray_camera"));
//				ros::spinOnce(); ros::Duration(0.1).sleep();

			for (size_t m = 0; m < models.size(); ++m)
			{
				const GazeboModel& model = *models[m];
				const GazeboModelState& state = states[m];

				Eigen::Isometry3d worldTshape = state.pose/* * body_pair.second->getPose()*/;
				Eigen::Isometry3d shapeTworld = worldTshape.inverse(Eigen::Isometry);

				const Eigen::Vector3d cam_body = shapeTworld * r0_world;
				const Eigen::Vector3d dir_body = shapeTworld.linear() * rn_world;

//					for (const auto& body_pair : model.bodies)
				{


//						tf::Transform temp;
//						Eigen::Isometry3d cameraTshape = cameraTworld * worldTshape;
//						tf::poseEigenToTF(cameraTshape, temp);
//						broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), cameraFrame, model.name + "_" + body_pair.first));
//						tf::poseEigenToTF(cameraTworld, temp);
//						broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), cameraFrame, worldFrame + "_inv"));

//						Eigen::Isometry3d bodyTcam_origin = Eigen::Isometry3d::Identity(); bodyTcam_origin.translation() = cam_body;
//						tf::poseEigenToTF(bodyTcam_origin, temp);
//						broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), model.name + "_" + body_pair.first, "ray_camera"));
//						ros::spinOnce(); ros::Duration(0.1).sleep();
//
//						Eigen::Isometry3d bodyTdirpt = Eigen::Isometry3d::Identity(); bodyTdirpt.translation() = cam_body + dir_body;
//						tf::poseEigenToTF(bodyTdirpt, temp);
//						broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), model.name + "_" + body_pair.first, "dir_" + model.name + "_" + body_pair.first));
//						ros::spinOnce(); ros::Duration(0.1).sleep();

					EigenSTL::vector_Vector3d intersections;
					if (mps::rayIntersectsModel(cam_body, dir_body, model, intersections))
					{
						Eigen::Vector3d closestIntersection;
						for (const Eigen::Vector3d& pt_body : intersections)
						{
//								Eigen::Isometry3d bodyTdirpt = Eigen::Isometry3d::Identity(); bodyTdirpt.translation() = pt_body;
//								tf::poseEigenToTF(bodyTdirpt, temp);
//								broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), "mocap_" + model.name, "intersect"));
//								ros::spinOnce(); ros::Duration(0.1).sleep();
//					    		const Eigen::Vector3d pt_world = worldTshape * pt_body;
							double dist = dir_body.dot(pt_body - cam_body);
							assert(dist >= 0.0);
							#pragma omp critical
							{
								auto& zBuf = depthBuf.at<DepthT>(v, u);
								if (dist < zBuf)
								{
									zBuf = dist;
									labels.at<LabelT>(v, u) = m + 10;
//										if (model.name.find("table") != std::string::npos)
//										{
//											labels.at<LabelT>(v, u) = 1;
//										}
									closestIntersection = pt_body;
								}
							}
						}
//							Eigen::Isometry3d cameraTray = Eigen::Isometry3d::Identity(); cameraTray.translation() = toPoint3D<Eigen::Vector3d>(u, v, 1.0, cameraModel).normalized() * depthBuf.at<DepthT>(v, u);
//							tf::poseEigenToTF(cameraTray, temp);
//							broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), cameraFrame, "ray_camera"));
//							ros::spinOnce(); ros::Duration(0.5).sleep();
					}

//						Eigen::Isometry3d worldTray = Eigen::Isometry3d::Identity(); worldTray.translation() = closestIntersection;
//						tf::poseEigenToTF(worldTray, temp);
//						broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), worldFrame, "ray"));
//						ros::spinOnce(); ros::Duration(0.1).sleep();

				}
			}
		}
	}
	return labels;
}
