//
// Created by arprice on 7/23/19.
//

#include "mps_interactive_segmentation/GazeboMocap.h"
#include "mps_interactive_segmentation/GazeboModel.h"
#include "mps_interactive_segmentation/paths.h"
#include "mps_interactive_segmentation/loading.h"

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

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "tinyxml.h"

#include <boost/algorithm/string.hpp>

#include <regex>
#include <iterator>
#include <mutex>

#define DEBUG_MESH_LOADING true

#define CV_VERSION_AT_LEAST(x,y,z) (CV_VERSION_MAJOR>x || (CV_VERSION_MAJOR>=x && \
                                   (CV_VERSION_MINOR>y || (CV_VERSION_MINOR>=y && \
                                                           CV_VERSION_REVISION>=z))))

using mps::GazeboModel;
using mps::parseModelURL;
using mps::parsePackageURL;

typedef actionlib::SimpleActionServer<mps_msgs::SegmentRGBDAction> GTServer;

template <typename PointT>
PointT toPoint3D(const float uIdx, const float vIdx, const float depthMeters, const image_geometry::PinholeCameraModel& cam)
{
	PointT pt;
	pt.x() = (uIdx - cam.cx()) * depthMeters / cam.fx();
	pt.y() = (vIdx - cam.cy()) * depthMeters / cam.fy();
	pt.z() = depthMeters;
	return pt;
}

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
	std::unique_ptr<ros::Subscriber> cam_sub;
	image_geometry::PinholeCameraModel cameraModel;
	std::string worldFrame = "table_surface";
	std::string cameraFrame;
	std::unique_ptr<tf::TransformListener> listener;
	std::unique_ptr<tf::TransformBroadcaster> broadcaster;
	std::unique_ptr<GTServer> actionServer;

	std::unique_ptr<image_transport::Publisher> segmentationPub;

	std::mutex stateMtx;
	std::vector<std::shared_ptr<GazeboModel>> models;
	std::vector<GazeboModelState> states;

	Segmenter(ros::NodeHandle& nh, std::vector<std::shared_ptr<GazeboModel>> shapeModels_) : models(std::move(shapeModels_))
	{
		listener = std::make_unique<tf::TransformListener>(ros::Duration(60.0));
		broadcaster = std::make_unique<tf::TransformBroadcaster>();
		cam_sub = std::make_unique<ros::Subscriber>(nh.subscribe<sensor_msgs::CameraInfo>("/kinect2_victor_head/hd/camera_info", 1, boost::bind(&Segmenter::cam_cb, this, _1)));

		image_transport::ImageTransport it(ros::NodeHandle("~"));
		segmentationPub = std::make_unique<image_transport::Publisher>(it.advertise("segmentation", 1));

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

	cv::Mat segment()
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

		if (segmentationPub->getNumSubscribers() > 0)
		{
			std_msgs::Header h; h.frame_id = cameraFrame; h.stamp = ros::Time::now();
			segmentationPub->publish(cv_bridge::CvImage(h, "bgr8", labelColorsMap).toImageMsg());
		}

//		cv::waitKey(1);
//		cv::imshow("segmentation", labelColorsMap);
//		cv::waitKey(0);

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

		cv::Mat labels = segment();

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
    ros::init(argc, argv, "gazebo_segmentation_gt");
    ros::NodeHandle pnh("~");

	ros::Publisher debugPub = pnh.advertise<visualization_msgs::MarkerArray>("debug_shapes", 10, true);
	visualization_msgs::MarkerArray debugShapes;

//	cv::namedWindow("segmentation", /*cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | */cv::WINDOW_GUI_EXPANDED);

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

#if DEBUG_MESH_LOADING
        debugPub.publish(debugShapes);
#endif

        mocap.getTransforms();
//        mocap.sendTransforms(false);

	    std::lock_guard<std::mutex> lock(seg.stateMtx);
		seg.states.clear();
	    for (const auto& shape : shapeModels)
	    {
	    	// TODO: Skip adding this shape based on the plugin camera info in world file
//	    	if (shape.name == "kinect2_victor_head")
//		    {
//	    		continue;
//		    }

		    for (const auto& pair : shape.second->bodies)
		    {
			    const tf::StampedTransform& stf = mocap.linkPoses.at({shape.second->name, pair.first});
//			    const tf::StampedTransform& stf = mocap.linkPoses.at({shape.second->name, "link"});

			    Eigen::Isometry3d T;
			    tf::transformTFToEigen(stf, T);
			    seg.states.push_back({T});

//			    std::cerr << T.matrix() << std::endl;
		    }

	    }
    }

    return 0;
}
