//
// Created by arprice on 7/23/19.
//

#include "mps_interactive_segmentation/GazeboMocap.h"

#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/body_operations.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf_conversions/tf_eigen.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "tinyxml.h"

#include <regex>
#include <iterator>

std::vector<double> splitParams(const std::string& /*text*/)
{
static std::regex ws_re("\\s+"); // whitespace

return std::vector<double>{};
}

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
//	cv::LUT(labels, colormap, output);

	cv::applyColorMap(labels, output, colormap);

	return output;
}

struct GazeboModel
{
	std::string name;

	// TODO: Make one for each link...
	std::map<std::string, bodies::BodyPtr> bodies;
};

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

	Segmenter()
	{
		ros::NodeHandle nh;
		listener = std::make_unique<tf::TransformListener>(ros::Duration(60.0));
		broadcaster = std::make_unique<tf::TransformBroadcaster>();
		cam_sub = std::make_unique<ros::Subscriber>(nh.subscribe<sensor_msgs::CameraInfo>("/kinect2_victor_head/hd/camera_info", 1, boost::bind(&Segmenter::cam_cb, this, _1)));
	}

	void cam_cb (const sensor_msgs::CameraInfoConstPtr& cam_msg)
	{
		cameraModel.fromCameraInfo(cam_msg);
		cameraFrame = cam_msg->header.frame_id;
//		cameraModel.rawRoi().x = 0;
//		cameraModel.rawRoi().y = 0;
	}

	cv::Mat segment(const std::vector<GazeboModel>& models, const std::vector<GazeboModelState>& states)
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
		Eigen::Isometry3d cameraTworld = worldTcamera.inverse(Eigen::Isometry);

		auto roi = cameraModel.rectifiedRoi();

		using LabelT = uint16_t;
		using DepthT = float;
		cv::Mat labels = cv::Mat::zeros(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_16U);
		cv::Mat depthBuf(cameraModel.cameraInfo().height, cameraModel.cameraInfo().width, CV_32F, std::numeric_limits<DepthT>::max());

//		labels.at<LabelT>(10, 10) = 25;
//		labels.at<LabelT>(100, 100) = 250;

//		//		double alpha = 0.75;
//		cv::Mat labelColorsMap = colorByLabel(labels);
//		labelColorsMap.setTo(0, 0 == labels);
////		labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*scene->cv_rgb_cropped.image;
//		cv::imshow("segmentation", labelColorsMap);
//		cv::waitKey(0);


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

		const int step = 1;
#pragma omp parallel for
		for (int v = roi.y; v < roi.y+roi.height; /*++v*/v+=step)
		{
			std::cerr << v << std::endl;
			for (int u = roi.x; u < roi.x + roi.width; /*++u*/u+=step)
			{
				const Eigen::Vector3d rn_world = worldTcamera.linear() * toPoint3D<Eigen::Vector3d>(u, v, 1.0, cameraModel).normalized();
				const Eigen::Vector3d r0_world = worldTcamera.translation();

				tf::Transform temp;
//				Eigen::Isometry3d worldTray = Eigen::Isometry3d::Identity(); worldTray.translation() = r0_world + rn_world;
//				tf::poseEigenToTF(worldTray, temp);
//				broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), worldFrame, "ray"));
//				ros::spinOnce(); ros::Duration(0.1).sleep();
//
//				Eigen::Isometry3d cameraTray = Eigen::Isometry3d::Identity(); cameraTray.translation() = toPoint3D<Eigen::Vector3d>(u, v, 1.0, cameraModel).normalized();
//				tf::poseEigenToTF(cameraTray, temp);
//				broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), cameraFrame, "ray_camera"));
//				ros::spinOnce(); ros::Duration(0.1).sleep();

				for (size_t m = 0; m < models.size(); ++m)
				{
					const GazeboModel& model = models[m];
					const GazeboModelState& state = states[m];

					for (const auto& body_pair : model.bodies)
					{
						Eigen::Isometry3d worldTshape = state.pose/* * body_pair.second->getPose()*/;
						Eigen::Isometry3d shapeTworld = worldTshape.inverse(Eigen::Isometry);

//						tf::Transform temp;
//						Eigen::Isometry3d cameraTshape = cameraTworld * worldTshape;
//						tf::poseEigenToTF(cameraTshape, temp);
//						broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), cameraFrame, model.name + "_" + body_pair.first));
//						tf::poseEigenToTF(cameraTworld, temp);
//						broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), cameraFrame, worldFrame + "_inv"));

						const Eigen::Vector3d cam_body = shapeTworld * r0_world;
						const Eigen::Vector3d dir_body = shapeTworld.linear() * rn_world;

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
						if (body_pair.second->intersectsRay(cam_body, dir_body, &intersections))
						{
							for (const Eigen::Vector3d &pt_body : intersections)
							{
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
									}
								}
							}
						}
					}
				}
			}
		}

//		double alpha = 0.75;
		cv::Mat labelColorsMap = colorByLabel(labels);
		labelColorsMap.setTo(0, 0 == labels);
//		labelColorsMap = alpha*labelColorsMap + (1.0-alpha)*scene->cv_rgb_cropped.image;
		cv::waitKey(1);
		cv::imshow("segmentation", labelColorsMap);
		cv::waitKey(0);


		return labels;
	}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gazebo_segmentation_gt");
    ros::NodeHandle pnh("~");

	cv::namedWindow("segmentation", /*cv::WINDOW_AUTOSIZE | cv::WINDOW_KEEPRATIO | */cv::WINDOW_GUI_EXPANDED);

    // Load Gazebo world file and get meshes
//	const std::string gazeboWorldFilename = "/home/kunhuang/catkin_ws/src/mps_interactive_segmentation/worlds/interseg_table_new.world";
	const std::string gazeboWorldFilename = "/home/kunhuang/catkin_ws/src/mps_interactive_segmentation/worlds/experiment_world.world";

	// TODO: Poses for shapes
	std::vector<GazeboModel> shapeModels;

	TiXmlDocument doc(gazeboWorldFilename);
	doc.LoadFile();

	TiXmlElement* root = doc.RootElement();
	if (!root)
	{
		ROS_FATAL_STREAM("Unable to open Gazebo world file '" << gazeboWorldFilename << "'. Unable to load object models.");
		return -1;
	}

//	TiXmlElement* xSdf = root->FirstChildElement("sdf");
	TiXmlElement* xWorld = root->FirstChildElement("world");
	TiXmlElement* xModel = xWorld->FirstChildElement("model");
	while (xModel)
	{
		std::string name(xModel->Attribute("name"));
		std::cerr << name << std::endl;

		GazeboModel mod;
		mod.name = name;

		// Loop through all links in model
		TiXmlElement* xLink = xModel->FirstChildElement("link");
		while (xLink)
		{
			std::string linkName(xLink->Attribute("name"));

			geometry_msgs::Pose pose;
			pose.orientation.w = 1;
			TiXmlElement* xPose = xLink->FirstChildElement("pose");
			if (xPose)
			{
				// TODO: Parse sequence as pose
				pose.position.z = 0.115;
				std::cerr << "Boo!" << std::endl;
			}

			TiXmlElement* xVisual = xLink->FirstChildElement("visual");
			if (!xVisual) { continue; }
			TiXmlElement* xGeometry = xVisual->FirstChildElement("geometry");
			if (!xGeometry) { continue; }

			TiXmlElement* xBox = xGeometry->FirstChildElement("box");
			TiXmlElement* xCylinder = xGeometry->FirstChildElement("cylinder");
			TiXmlElement* xMesh = xGeometry->FirstChildElement("mesh");
			// TODO: Plane (shape_msgs::Plane)

			if (xBox)
			{
				// TODO: Use split() on getText; write primitive.dimensions
				std::cerr << "Is Box!" << std::endl;
				shape_msgs::SolidPrimitive primitive;
				primitive.type = shape_msgs::SolidPrimitive::BOX;
//				primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] =
				std::cerr << xBox->FirstChildElement("size")->GetText() << std::endl;

			}
			else if (xCylinder)
			{
				std::cerr << "Is Cylinder!" << std::endl;
				shape_msgs::SolidPrimitive primitive;
				primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
				primitive.dimensions.resize(2);
				char* end_ptr;
				primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = std::strtod(xCylinder->FirstChildElement("length")->GetText(), &end_ptr);
				primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = std::strtod(xCylinder->FirstChildElement("radius")->GetText(), &end_ptr);

//				shapes::Cylinder* s = dynamic_cast<shapes::Cylinder*>(shapes::constructShapeFromMsg(primitive));
//				s->print(std::cerr);
				mod.bodies.insert({linkName, bodies::BodyPtr(bodies::constructBodyFromMsg(primitive, pose))});
				shapeModels.push_back(mod);
			}
			else if (xMesh)
			{
				std::cerr << "Is Mesh!" << std::endl;
			}
			else
			{
				ROS_WARN_STREAM("Unknown shape type: " << name << ".");
			}


			xLink = xLink->NextSiblingElement("link");
		}


		xModel = xModel->NextSiblingElement("model");
	}

    mps::GazeboMocap mocap;

    std::vector<std::string> models = mocap.getModelNames();

    for (auto& model : models)
    {
        mocap.markerOffsets.insert({{model, std::string("link")}, tf::StampedTransform(tf::Transform::getIdentity(), ros::Time(0), mocap.mocap_frame_id, model)});
    }

	Segmenter seg;
    seg.worldFrame = mocap.gazeboTmocap.frame_id_;

    ros::Rate r(10.0);
    while(ros::ok())
    {
        r.sleep();
        ros::spinOnce();

        mocap.getTransforms();
//        mocap.sendTransforms(false);

	    std::vector<GazeboModelState> states;
	    for (const auto& shape : shapeModels)
	    {
	    	for (const auto& pair : shape.bodies)
		    {
			    const tf::StampedTransform& stf = mocap.linkPoses.at({shape.name, pair.first});

			    Eigen::Isometry3d T;
			    tf::transformTFToEigen(stf, T);
			    states.push_back({T});

			    std::cerr << T.matrix() << std::endl;
		    }

	    }


        seg.segment(shapeModels, states);
    }

    return 0;
}