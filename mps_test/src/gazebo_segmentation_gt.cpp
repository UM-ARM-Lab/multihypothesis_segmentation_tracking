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

#define VOXELIZER_ROS 0
#define VOXELIZER_GVDB 1

#include "mps_simulation/GazeboMocap.h"
#include "mps_simulation/GazeboModel.h"
#include "mps_simulation/paths.h"
#include "mps_simulation/loading.h"
#include "../include/mps_test/SceneVoxelizer.h"
#if VOXELIZER == VOXELIZER_GVDB
#include "mps_simulation/GVDBVoxelizer.h"
#else
#include "../include/mps_test/ROSVoxelizer.h"
#endif

#include <mps_voxels/VoxelRegion.h>
#include <mps_voxels/parameters/VoxelRegionBuilder.hpp>
#include <mps_voxels/visualization/visualize_voxel_region.h>
#include <mps_voxels/OccupancyData.h>
#include <mps_voxels/logging/DataLog.h>
#include <mps_voxels/logging/log_occupancy_data.h>

#include "geometric_shapes/shapes.h"

#include <actionlib/server/simple_action_server.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <mps_msgs/SegmentRGBDAction.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/imgproc.hpp>

#include "tinyxml.h"

#include <mutex>

#ifndef DEBUG_MESH_LOADING
#define DEBUG_MESH_LOADING false
#endif

#define CV_VERSION_AT_LEAST(x,y,z) (CV_VERSION_MAJOR>x || (CV_VERSION_MAJOR>=x && \
                                   (CV_VERSION_MINOR>y || (CV_VERSION_MINOR>=y && \
                                                           CV_VERSION_REVISION>=z))))

using mps::GazeboModel;
using mps::SceneVoxelizer;
using mps::parseModelURL;
using mps::parsePackageURL;

typedef actionlib::SimpleActionServer<mps_msgs::SegmentRGBDAction> GTServer;

const bool shouldLog = true;

template <typename PointT>
PointT toPoint3D(const float uIdx, const float vIdx, const float depthMeters, const image_geometry::PinholeCameraModel& cam)
{
	PointT pt;
	pt.x() = (uIdx - cam.cx()) * depthMeters / cam.fx();
	pt.y() = (vIdx - cam.cy()) * depthMeters / cam.fy();
	pt.z() = depthMeters;
	return pt;
}

// TODO: colormap or mt ref
cv::Mat colorByLabel(const cv::Mat& input)
{
	double min;
	double max;
	cv::minMaxIdx(input, &min, &max);
	cv::Mat labels;
	input.convertTo(labels, CV_8UC1);

	cv::Mat colormap(256, 1, CV_8UC3);
	cv::randu(colormap, 0, 256);

	cv::Mat output;
#if CV_VERSION_AT_LEAST(3, 3, 0)
	cv::applyColorMap(labels, output, colormap);
#else
	cv::LUT(labels, colormap, output);
#endif

	return output;
}

struct GazeboModelState
{
	using Pose = Eigen::Isometry3d;

	Pose pose;

	enum { NeedsToAlign = (sizeof(Pose)%16)==0 };
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
};

class Segmenter
{
public:
	std::default_random_engine re;
	std::unique_ptr<ros::Subscriber> cam_sub;
	image_geometry::PinholeCameraModel cameraModel;
	std::string worldFrame = "table_surface";
	std::string tableFrame = "table_surface";
	std::string cameraFrame;

	std::unique_ptr<tf::TransformListener> listener;
	std::unique_ptr<tf::TransformBroadcaster> broadcaster;
	std::unique_ptr<GTServer> actionServer;
	std::unique_ptr<SceneVoxelizer> voxelizer;

	std::unique_ptr<image_transport::Publisher> imageSegmentationPub;
	std::unique_ptr<ros::Publisher> voxelSegmentationPub;

	std::mutex stateMtx;
	std::vector<std::shared_ptr<GazeboModel>> models;
	std::vector<GazeboModelState> states;

	Segmenter(ros::NodeHandle& nh, std::vector<std::shared_ptr<GazeboModel>> shapeModels_)
		: re(std::random_device()()),
		  models(std::move(shapeModels_))
	{
		listener = std::make_unique<tf::TransformListener>(ros::Duration(60.0));
		broadcaster = std::make_unique<tf::TransformBroadcaster>();
		cam_sub = std::make_unique<ros::Subscriber>(nh.subscribe<sensor_msgs::CameraInfo>("/kinect2_victor_head/hd/camera_info", 1, boost::bind(&Segmenter::cam_cb, this, _1)));

		image_transport::ImageTransport it(ros::NodeHandle("~"));
		imageSegmentationPub = std::make_unique<image_transport::Publisher>(it.advertise("image_segmentation", 1));
		voxelSegmentationPub = std::make_unique<ros::Publisher>(nh.advertise<visualization_msgs::MarkerArray>("voxel_segmentation", 1, true));

		actionServer = std::make_unique<GTServer>(nh, "gazebo_segmentation", boost::bind(&Segmenter::execute, this, _1), false);
//		actionServer->start();
	}

	void cam_cb (const sensor_msgs::CameraInfoConstPtr& cam_msg)
	{
		cameraModel.fromCameraInfo(cam_msg);
		cameraFrame = cam_msg->header.frame_id;
//		cameraModel.rawRoi().x = 0;
//		cameraModel.rawRoi().y = 0;
	}

	cv::Mat segmentUVD()
	{
		if (cameraFrame.empty())
		{
			ROS_INFO_STREAM("No camera seen yet.");
			return cv::Mat();
		}

		tf::StampedTransform cameraPose;
		if (!listener->waitForTransform(worldFrame, cameraFrame, ros::Time(0), ros::Duration(5.0)))
		{
			ROS_WARN_STREAM("Failed to look up transform between '" << worldFrame << "' and '" << cameraFrame << "'.");
			return cv::Mat();
		}
		listener->lookupTransform(worldFrame, cameraFrame, ros::Time(0), cameraPose);

		Eigen::Isometry3d worldTcamera;
		tf::transformTFToEigen(cameraPose, worldTcamera);

		auto roi = cameraModel.rectifiedRoi();

		using LabelT = uint16_t;
		using DepthT = double;
		cv::Mat labels = cv::Mat::zeros(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_16U);
		cv::Mat depthBuf(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_64F, std::numeric_limits<DepthT>::max());


//		for (size_t m = 0; m < models.size(); ++m)
//		{
//			const GazeboModel &model = models[m];
//			const GazeboModelState &state = states[m];
//
//			tf::Transform temp;
//
//			tf::poseEigenToTF(state.pose, temp);
//			broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), worldFrame, model.name + "__base"));
//			ros::spinOnce(); ros::Duration(0.1).sleep();
//
//			for (const auto& body_pair : model.bodies)
//			{
//				std::cerr << body_pair.second->getPose().matrix() << std::endl;
//				Eigen::Isometry3d worldTshape = state.pose * body_pair.second->getPose();
//
//
//				tf::poseEigenToTF(worldTshape, temp);
//				broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), worldFrame, model.name + "_" + body_pair.first));
//				ros::spinOnce(); ros::Duration(0.1).sleep();
//
//				Eigen::Isometry3d cameraTshape = cameraTworld * worldTshape;
//
//				tf::poseEigenToTF(cameraTshape, temp);
//				broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), cameraFrame, model.name + "_" + body_pair.first));
//				ros::spinOnce(); ros::Duration(0.1).sleep();
//
//			}
//		}

		std::lock_guard<std::mutex> lock(stateMtx);
		if (states.size() != models.size())
		{
			throw std::logic_error("Model/State size mismatch!");
		}

		const int step = 1;

#pragma omp parallel for
		for (int v = roi.y; v < roi.y+roi.height; /*++v*/v+=step)
		{
//			if (actionServer->isActive())
//			{
//				mps_msgs::SegmentRGBDFeedback fb;
//				fb.progress = static_cast<float>(v)/roi.height;
//				actionServer->publishFeedback(fb);
//			}
//			std::cerr << v << std::endl;

			for (int u = roi.x; u < roi.x + roi.width; /*++u*/u+=step)
			{
				const Eigen::Vector3d rn_world = worldTcamera.linear() * toPoint3D<Eigen::Vector3d>(u, v, 1.0, cameraModel).normalized();
				const Eigen::Vector3d r0_world = worldTcamera.translation();

				tf::Transform temp;
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
							for (const Eigen::Vector3d &pt_body : intersections)
							{
//								Eigen::Isometry3d bodyTdirpt = Eigen::Isometry3d::Identity(); bodyTdirpt.translation() = pt_body;
//								tf::poseEigenToTF(bodyTdirpt, temp);
//								broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), "mocap_" + model.name, "intersect"));
//								ros::spinOnce(); ros::Duration(0.1).sleep();
//							const Eigen::Vector3d pt_world = worldTshape * pt_body;
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

//		double alpha = 0.75;
		cv::Mat labelColorsMap = colorByLabel(labels);
		labelColorsMap.setTo(0, 0 == labels);
//		labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*scene->cv_rgb_cropped.image;

		if (imageSegmentationPub->getNumSubscribers() > 0)
		{
			std_msgs::Header h; h.frame_id = cameraFrame; h.stamp = ros::Time::now();
			imageSegmentationPub->publish(cv_bridge::CvImage(h, "bgr8", labelColorsMap).toImageMsg());
		}

//		cv::waitKey(1);
//		cv::imshow("segmentation", labelColorsMap);
//		cv::waitKey(0);

		return labels;
	}

	mps::VoxelRegion::VertexLabels segmentXYZ(const mps::VoxelRegion& region)
	{
		tf::StampedTransform tablePose;
		if (!listener->waitForTransform(worldFrame, tableFrame, ros::Time(0), ros::Duration(5.0)))
		{
			ROS_WARN_STREAM("Failed to look up transform between '" << worldFrame << "' and '" << tableFrame << "'.");
			return cv::Mat();
		}
		listener->lookupTransform(worldFrame, tableFrame, ros::Time(0), tablePose);

		Eigen::Isometry3d worldTtable;
		tf::transformTFToEigen(tablePose, worldTtable);

		std::lock_guard<std::mutex> lock(stateMtx);
		if (states.size() != models.size())
		{
			throw std::logic_error("Model/State size mismatch!");
		}

		if (!this->voxelizer)
		{
			#if VOXELIZER == VOXELIZER_GVDB
			std::vector<const shapes::Mesh*> meshes;
			std::vector<Eigen::Isometry3d> poses;
			for (const auto& m : this->models)
			{
				for (const auto& s : m->shapes)
				{
					const auto* mesh = dynamic_cast<const shapes::Mesh*>(s.second.get());
					if (mesh) { meshes.push_back(mesh); poses.push_back(Eigen::Isometry3d::Identity()); }
				}
			}
			this->voxelizer = std::make_unique<mps::GVDBVoxelizer>(region, meshes);
			#else
			this->voxelizer = std::make_unique<mps::ROSVoxelizer>(models);
			#endif
		}

		std::vector<Eigen::Isometry3d> poses;
		for (size_t m = 0; m < models.size(); ++m)
		{
			poses.emplace_back(worldTtable * states[m].pose);
		}

		mps::VoxelRegion::VertexLabels labels = voxelizer->voxelize(region, poses);

		if (voxelSegmentationPub->getNumSubscribers() > 0)
		{
			std_msgs::Header h; h.frame_id = tableFrame; h.stamp = ros::Time::now();
			auto viz = mps::visualize(region, labels, h, re);
			voxelSegmentationPub->publish(viz);
		}

		return labels;
	}

	void execute(const mps_msgs::SegmentRGBDGoalConstPtr& /*goal*/)
	{
		std::cerr << "Executing segmentation callback." << std::endl;
		mps_msgs::SegmentRGBDResult result_;

		if (cameraFrame.empty())
		{
			const std::string err = "No camera data.";
			ROS_ERROR_STREAM(err);
			actionServer->setAborted(result_, err);
			return;
		}

		if (states.empty())
		{
			const std::string err = "No state data.";
			ROS_ERROR_STREAM(err);
			actionServer->setAborted(result_, err);
			return;
		}

		cv::Mat labels = segmentUVD();

		if (labels.empty())
		{
			const std::string err = "Labelling failed.";
			ROS_ERROR_STREAM(err);
			actionServer->setAborted(result_, err);
			return;
		}

		sensor_msgs::Image seg;

		std_msgs::Header header; // empty header
		header.stamp = ros::Time::now(); // time
		cv_bridge::CvImage img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::MONO16, labels);
		img_bridge.toImageMsg(seg); // from cv_bridge to sensor_msgs::Image

		result_.segmentation = seg;
		actionServer->setSucceeded(result_);
	}

};

int main(int argc, char** argv)
{
	std::shared_ptr<mps::VoxelRegion> region = std::make_shared<mps::VoxelRegion>(mps::VoxelRegionBuilder::build(YAML::Load("{roi: {min: {x: -0.4, y: -0.6, z: -0.020}, max: {x: 0.4, y: 0.6, z: 0.5}, resolution: 0.01}}")));

	ros::init(argc, argv, "gazebo_segmentation_gt");
	ros::NodeHandle pnh("~");

	std::string worldFileParam;
	if (!pnh.getParam("/simulation_world", worldFileParam))
	{
		ROS_FATAL_STREAM("Unable to read world file name from parameter server '" << worldFileParam << "'");
		return -1;
	}
	// Load Gazebo world file and get meshes
	const std::string gazeboWorldFilename = parsePackageURL(worldFileParam);

	std::vector<std::string> modelsToIgnore;
	pnh.param("ignore", modelsToIgnore, std::vector<std::string>());
	std::set<std::string> modsToIgnore(modelsToIgnore.begin(), modelsToIgnore.end());
	if (!modelsToIgnore.empty())
	{
		std::cerr << "Ignoring models:" << std::endl;
		for (const auto& m : modelsToIgnore)
		{
			std::cerr << "\t" << m << std::endl;
		}
	}

	std::map<std::string, std::shared_ptr<GazeboModel>> shapeModels = mps::getWorldFileModels(gazeboWorldFilename, modsToIgnore);
	if (shapeModels.empty())
	{
		ROS_WARN_STREAM("No models were loaded from world file '" << gazeboWorldFilename << "'.");
	}

	mps::GazeboMocap mocap;

	std::vector<std::string> models = mocap.getModelNames();

	for (auto& modelName : models)
	{
		auto iter = shapeModels.find(modelName);
		if (iter != shapeModels.end()  && iter->second && !iter->second->shapes.empty())
		{
			for (const auto& linkPair : iter->second->shapes)
			{
				mocap.markerOffsets.insert({{modelName, linkPair.first}, tf::StampedTransform(tf::Transform::getIdentity(), ros::Time(0), mocap.mocap_frame_id, modelName)});
			}
		}
	}

	for (auto& shape : shapeModels)
	{
		shape.second->computeBoundingSphere();
	}

	std::vector<std::shared_ptr<GazeboModel>> tmpModels; tmpModels.reserve(shapeModels.size());
	for (const auto& m : shapeModels) { tmpModels.push_back(m.second); }
	Segmenter seg(pnh, tmpModels);
	seg.worldFrame = mocap.gazeboTmocap.frame_id_;


	// Wait for camera
	ros::Duration(1.0).sleep();
	ros::spinOnce();
	seg.actionServer->start();

	ros::Rate r(10.0);
	while(ros::ok())
	{
		r.sleep();
		ros::spinOnce();

		mocap.getTransforms();
//		mocap.sendTransforms(false);
		{
			std::lock_guard<std::mutex> lock(seg.stateMtx);
			seg.states.clear();
			for (const auto& shape : shapeModels)
			{
				// TODO: Skip adding this shape based on the plugin camera info in world file
//			if (shape.name == "kinect2_victor_head")
//			{
//				continue;
//			}

				for (const auto& pair : shape.second->bodies)
				{
					const tf::StampedTransform& stf = mocap.linkPoses.at({shape.second->name, pair.first});
//				const tf::StampedTransform& stf = mocap.linkPoses.at({shape.second->name, "link"});

					Eigen::Isometry3d T;
					tf::transformTFToEigen(stf, T);
					seg.states.push_back({T});

//				std::cerr << T.matrix() << std::endl;
				}

			}
		}

		mps::OccupancyData occupancy(region);
		occupancy.vertexState = seg.segmentXYZ(*region);
		if (occupancy.vertexState.size() != occupancy.voxelRegion->num_vertices()) { throw std::logic_error("Fake news!"); }
//		for (const auto& v : occupancy.vertexState)
//		{
//			if (v != mps::VoxelRegion::FREE_SPACE)
//			{
//				std::cerr << "Yqy!" << std::endl;
//			}
//		}
		if (shouldLog)
		{
			mps::DataLog logger("/tmp/gazebo_segmentation/gt_occupancy.bag");
			logger.activeChannels.insert("gt");
			logger.log<mps::OccupancyData>("gt", occupancy);
		}
		ros::shutdown();
		exit(0);

	}

	return 0;
}
