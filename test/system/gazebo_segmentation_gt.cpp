//
// Created by arprice on 7/23/19.
//

#include "mps_interactive_segmentation/GazeboMocap.h"
#include "mps_interactive_segmentation/GazeboModel.h"
#include "mps_interactive_segmentation/paths.h"

#include "geometric_shapes/shapes.h"
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_to_marker.h>

#include <actionlib/server/simple_action_server.h>

#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <mps_msgs/SegmentRGBDAction.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

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

#define DEBUG_MESH_LOADING true

#define CV_VERSION_AT_LEAST(x,y,z) (CV_VERSION_MAJOR>x || (CV_VERSION_MAJOR>=x && \
                                   (CV_VERSION_MINOR>y || (CV_VERSION_MINOR>=y && \
                                                           CV_VERSION_REVISION>=z))))

using mps::GazeboModel;
using mps::parseModelURL;
using mps::parsePackageURL;

typedef actionlib::SimpleActionServer<mps_msgs::SegmentRGBDAction> GTServer;

std::vector<double> splitParams(const std::string& text) //one more element at the last
{
	std::vector<double> result;
	std::stringstream ss;
	ss << text;
	while (ss)
	{
		double temp;
		ss >> temp;
		result.push_back(temp);
	}
	return result;
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

	std::vector<GazeboModel> models;
	std::vector<GazeboModelState> states;

	Segmenter(ros::NodeHandle& nh, std::vector<GazeboModel> shapeModels_) : models(std::move(shapeModels_))
	{
		listener = std::make_unique<tf::TransformListener>(ros::Duration(60.0));
		broadcaster = std::make_unique<tf::TransformBroadcaster>();
		cam_sub = std::make_unique<ros::Subscriber>(nh.subscribe<sensor_msgs::CameraInfo>("/kinect2_victor_head/hd/camera_info", 1, boost::bind(&Segmenter::cam_cb, this, _1)));

		actionServer = std::make_unique<GTServer>(nh, "gazebo_segmentation", boost::bind(&Segmenter::execute, this, _1), false);
		actionServer->start();
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
//		Eigen::Isometry3d cameraTworld = worldTcamera.inverse(Eigen::Isometry);

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

		// TODO: Remove after debugging
//		roi.y = 750; roi.height = 100;
//		roi.x = 1400; roi.width = 100;



#pragma omp parallel for
		for (int v = roi.y; v < roi.y+roi.height; /*++v*/v+=step)
		{
//			if (actionServer->isActive())
//			{
//				mps_msgs::SegmentRGBDFeedback fb;
//				fb.progress = static_cast<float>(v)/roi.height;
//				actionServer->publishFeedback(fb);
//			}
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

//				Eigen::Isometry3d cameraTray = Eigen::Isometry3d::Identity(); cameraTray.translation() = toPoint3D<Eigen::Vector3d>(u, v, 1.0, cameraModel).normalized();
//				tf::poseEigenToTF(cameraTray, temp);
//				broadcaster->sendTransform(tf::StampedTransform(temp, ros::Time::now(), cameraFrame, "ray_camera"));
//				ros::spinOnce(); ros::Duration(0.1).sleep();

				for (size_t m = 0; m < models.size(); ++m)
				{
					const GazeboModel& model = models[m];
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
										if (model.name.find("table") != std::string::npos)
										{
											labels.at<LabelT>(v, u) = 1;
										}
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
		cv::waitKey(1);
		cv::imshow("segmentation", labelColorsMap);
		cv::waitKey(0);


		return labels;
	}

	void execute(const mps_msgs::SegmentRGBDGoalConstPtr& /*goal*/)
	{
		mps_msgs::SegmentRGBDResult result_;

		if (cameraFrame.empty())
		{
			actionServer->setAborted(result_, "No camera data.");
		}

		if (states.empty())
		{
			actionServer->setAborted(result_, "No state data.");
		}

		cv::Mat labels = segment();

		if (labels.empty())
		{
			actionServer->setAborted(result_, "Labelling failed.");
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

    // Load Gazebo world file and get meshes
	const std::string gazeboWorldFilename = parsePackageURL("package://mps_interactive_segmentation/worlds/experiment_world.world");

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
	TiXmlElement* xModState = xWorld->FirstChildElement("state");

	while (xModel)
	{
		std::string name(xModel->Attribute("name"));
		std::cerr << name << std::endl;

		GazeboModel mod;
		mod.name = name;

		// skip loading camera and table
		if (name == "kinect2_victor_head" /*|| name == "table"*/)
		{
			xModel = xModel->NextSiblingElement("model");
			continue;
		}

		// Loop through all links in model
		TiXmlElement* xLink = xModel->FirstChildElement("link");
		while (xLink)
		{
			std::string linkName(xLink->Attribute("name"));

			Eigen::Isometry3d eigenPose;
			geometry_msgs::Pose pose;
			pose.orientation.w = 1;
			TiXmlElement* xPose = xLink->FirstChildElement("pose");
			if (xPose)
			{
				std::cerr << xPose->GetText() << std::endl;
				std::vector<double> posexyz = splitParams(xPose->GetText());
//				assert(posexyz.size() == 6);

				// Load scale
				std::vector<double> modScale (3,1.0);
				TiXmlElement* xMods = xModState->FirstChildElement("model");
				while (true)
				{
					if (!xMods)
					{
						std::cerr << "No corresponding scale found!" << std::endl;
						break;
					}
					else if(xMods->Attribute("name") == mod.name)
					{
						std::cerr << "scale: " << xMods->FirstChildElement("scale")->GetText() << std::endl;
						std::vector<double> temp = splitParams(xMods->FirstChildElement("scale")->GetText());
						modScale[0] = temp[0];
						modScale[1] = temp[1];
						modScale[2] = temp[2];
						break;
					}
					xMods = xMods->NextSiblingElement("model");
				}

				pose.position.x = posexyz[0] * modScale[0];
				pose.position.y = posexyz[1] * modScale[1];
				pose.position.z = posexyz[2] * modScale[2];

				Eigen::Quaterniond q(Eigen::AngleAxisd(posexyz[3], Eigen::Vector3d::UnitZ())
			                       * Eigen::AngleAxisd(posexyz[4], Eigen::Vector3d::UnitY())
				                   * Eigen::AngleAxisd(posexyz[5], Eigen::Vector3d::UnitX()));

				pose.orientation.x = q.x();
				pose.orientation.y = q.y();
				pose.orientation.z = q.z();
				pose.orientation.w = q.w();

				eigenPose.translation() = Eigen::Map<Eigen::Vector3d>(posexyz.data());
				eigenPose.linear() = q.matrix();
			}

			TiXmlElement* xVisual = xLink->FirstChildElement("visual");
			if (!xVisual) { continue; }
			TiXmlElement* xGeometry = xVisual->FirstChildElement("geometry");
			if (!xGeometry) { continue; }

			TiXmlElement* xBox = xGeometry->FirstChildElement("box");
			TiXmlElement* xCylinder = xGeometry->FirstChildElement("cylinder");
			TiXmlElement* xMesh = xGeometry->FirstChildElement("mesh");
			TiXmlElement* xPlane = xGeometry->FirstChildElement("plane");

			if (xBox)
			{
				std::cerr << "Is Box!" << std::endl;
				shape_msgs::SolidPrimitive primitive;
				primitive.type = shape_msgs::SolidPrimitive::BOX;
				primitive.dimensions.resize(3);
				std::vector<double> BoxXYZ = splitParams(xBox->FirstChildElement("size")->GetText());

				// Load scale
				std::vector<double> modScale (3,1.0);
				TiXmlElement* xMods = xModState->FirstChildElement("model");
				while (true)
				{
					if (!xMods)
					{
						std::cerr << "No corresponding scale found!" << std::endl;
						break;
					}
					else if(xMods->Attribute("name") == mod.name)
					{
						std::cerr << "scale: " << xMods->FirstChildElement("scale")->GetText() << std::endl;
						std::vector<double> temp = splitParams(xMods->FirstChildElement("scale")->GetText());
						modScale[0] = temp[0];
						modScale[1] = temp[1];
						modScale[2] = temp[2];
						break;
					}
					xMods = xMods->NextSiblingElement("model");
				}

				primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = BoxXYZ[0] * modScale[0];
				primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = BoxXYZ[1] * modScale[1];
				primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = BoxXYZ[2] * modScale[2];

				shapes::ShapePtr shapePtr(shapes::constructShapeFromMsg(primitive));

				if (name.find("table") != std::string::npos)
				{
					std::cerr << "table pose: " << pose.position.x << " " << pose.position.y << " " << pose.position.z << std::endl;
					pose.position.z += 1; // why??? The <pose> should be put in the correct place in .world
				}

#if DEBUG_MESH_LOADING
				visualization_msgs::Marker marker;
				shapes::constructMarkerFromShape(shapePtr.get(), marker, true);
				// Do some more marker stuff here
				marker.color.a = 1.0;
				marker.color.r=rand()/(float)RAND_MAX;
				marker.color.g=rand()/(float)RAND_MAX;
				marker.color.b=rand()/(float)RAND_MAX;
				marker.pose = pose;
				marker.header.frame_id = "mocap_" + name; //"table_surface";
				marker.header.stamp = ros::Time::now();
				marker.ns = name;
				marker.id = 1;
				marker.frame_locked = true;
				debugShapes.markers.push_back(marker);
#endif

				mod.shapes.insert({linkName, shapePtr});
				mod.bodies.insert({linkName, bodies::BodyPtr(bodies::constructBodyFromMsg(primitive, pose))});
				shapeModels.push_back(mod);
			}
			else if (xCylinder)
			{
				std::cerr << "Is Cylinder!" << std::endl;
				shape_msgs::SolidPrimitive primitive;
				primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
				primitive.dimensions.resize(2);
				char* end_ptr;

				// Load scale
				std::vector<double> modScale (3,1.0);
				TiXmlElement* xMods = xModState->FirstChildElement("model");
				while (true)
				{
					if (!xMods)
					{
						std::cerr << "No corresponding scale found!" << std::endl;
						break;
					}
					else if(xMods->Attribute("name") == mod.name)
					{
						std::cerr << "scale: " << xMods->FirstChildElement("scale")->GetText() << std::endl;
						std::vector<double> temp = splitParams(xMods->FirstChildElement("scale")->GetText());
						modScale[0] = temp[0];
						modScale[1] = temp[1];
						modScale[2] = temp[2];
						break;
					}
					xMods = xMods->NextSiblingElement("model");
				}

				primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = std::strtod(xCylinder->FirstChildElement("length")->GetText(), &end_ptr) * modScale[2];
				primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = std::strtod(xCylinder->FirstChildElement("radius")->GetText(), &end_ptr) * std::max(modScale[0], modScale[1]);

				shapes::ShapePtr shapePtr(shapes::constructShapeFromMsg(primitive));

#if DEBUG_MESH_LOADING
				visualization_msgs::Marker marker;
				shapes::constructMarkerFromShape(shapePtr.get(), marker, true);
				// Do some more marker stuff here
				marker.color.a = 1.0;
				marker.color.r=rand()/(float)RAND_MAX;
				marker.color.g=rand()/(float)RAND_MAX;
				marker.color.b=rand()/(float)RAND_MAX;
				marker.pose = pose;
				marker.header.frame_id = "mocap_" + name; //"table_surface";
				marker.header.stamp = ros::Time::now();
				marker.ns = name;
				marker.id = 1;
				marker.frame_locked = true;
				debugShapes.markers.push_back(marker);
#endif

//				shapes::Cylinder* s = dynamic_cast<shapes::Cylinder*>(shapes::constructShapeFromMsg(primitive));
//				s->print(std::cerr);
				mod.shapes.insert({linkName, shapePtr});
				mod.bodies.insert({linkName, bodies::BodyPtr(bodies::constructBodyFromMsg(primitive, pose))});
				shapeModels.push_back(mod);
			}
			else if (xMesh)
			{
				std::cerr << "Is Mesh!" << std::endl;
				std::string pathuri = xMesh->FirstChildElement("uri")->GetText();

				const std::string model_path = parseModelURL(pathuri);

				shapes::Mesh* m = shapes::createMeshFromResource("file://" + model_path);
				shapes::ShapePtr shapePtr(m);
				if (name.find("coke_can") != std::string::npos)
				{
					for (unsigned i = 0; i < 3 * m->vertex_count; ++i)
					{
						m->vertices[i] /= 1000.0;
					}
				}
				if (name.find("disk_part") != std::string::npos)
				{
					xLink = xLink->NextSiblingElement("link"); continue;
					for (unsigned i = 0; i < 3 * m->vertex_count; ++i)
					{
						m->vertices[i] /= 1000.0;
					}
				}
				if (name.find("hammer") != std::string::npos)
				{
					for (unsigned i = 0; i < 3 * m->vertex_count; ++i)
					{
						m->vertices[i] *= 0.0254;
					}
				}

				// Load scale
				std::vector<double> modScale (3,1.0);
				TiXmlElement* xMods = xModState->FirstChildElement("model");
				while (true)
				{
					if (!xMods)
					{
						std::cerr << "No corresponding scale found!" << std::endl;
						break;
					}
					else if(xMods->Attribute("name") == mod.name)
					{
						std::cerr << "scale: " << xMods->FirstChildElement("scale")->GetText() << std::endl;
						std::vector<double> temp = splitParams(xMods->FirstChildElement("scale")->GetText());
						modScale[0] = temp[0];
						modScale[1] = temp[1];
						modScale[2] = temp[2];
						break;
					}
					xMods = xMods->NextSiblingElement("model");
				}
				for (unsigned i = 0; i <  m->vertex_count; ++i)
				{
					m->vertices[3 * i] *= modScale[0];
					m->vertices[3 * i + 1] *= modScale[1];
					m->vertices[3 * i + 2] *= modScale[2];
				}

				m->computeTriangleNormals();
				m->computeVertexNormals();
				std::cerr << "Vertices: " << m->vertex_count << ", Faces: " << m->triangle_count << std::endl;

				shapes::ShapeMsg mesh;
				constructMsgFromShape(m, mesh);
				bodies::BodyPtr bodyPtr(bodies::constructBodyFromMsg(mesh, pose));

				auto* cvx = dynamic_cast<bodies::ConvexMesh*>(bodyPtr.get());
				if (cvx) { std::cerr << "Converted to convex mesh. :(" << std::endl; }

#if DEBUG_MESH_LOADING
				visualization_msgs::Marker marker;
				shapes::constructMarkerFromShape(m, marker, true);
				// Do some more marker stuff here
				marker.color.a = 1.0;
				marker.color.r=rand()/(float)RAND_MAX;
				marker.color.g=rand()/(float)RAND_MAX;
				marker.color.b=rand()/(float)RAND_MAX;
				marker.pose = pose;
				marker.header.frame_id = "mocap_" + name; //"table_surface";
				marker.header.stamp = ros::Time::now();
				marker.ns = name;
				marker.id = 1;
				marker.frame_locked = true;
				debugShapes.markers.push_back(marker);
#endif

				mod.shapes.insert({linkName, shapePtr});
				mod.bodies.insert({linkName, bodyPtr});
				shapeModels.push_back(mod);
			}
			else if (xPlane)
			{
				std::cerr << "Is Plane!" << std::endl;
//				std::vector<double> planeNormal = splitParams(xPlane->FirstChildElement("normal")->GetText());
//				shape_msgs::Plane plane;
//				plane.coef[0] = planeNormal[0];
//				plane.coef[1] = planeNormal[1];
//				plane.coef[2] = planeNormal[2];
//				plane.coef[3] = 0.0;
//				mod.bodies.insert({linkName, bodies::BodyPtr(bodies::constructBodyFromMsg(plane, pose))});
//				shapeModels.push_back(mod);
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
		// TODO: load link name instead of "link"
		std::string linkname = "link";
//		if (model == "kinect2_victor_head")
//		{
//			linkname = "kinect2_victor_head";
//		}
        mocap.markerOffsets.insert({{model, linkname}, tf::StampedTransform(tf::Transform::getIdentity(), ros::Time(0), mocap.mocap_frame_id, model)});
    }

	for (auto& shape : shapeModels)
	{
		shape.computeBoundingSphere();
	}

	Segmenter seg(pnh, shapeModels);
    seg.worldFrame = mocap.gazeboTmocap.frame_id_;

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

		seg.states.clear();
	    for (const auto& shape : shapeModels)
	    {
	    	// TODO: Skip adding this shape based on the plugin camera info in world file
//	    	if (shape.name == "kinect2_victor_head")
//		    {
//	    		continue;
//		    }

	    	for (const auto& pair : shape.bodies)
		    {
//			    const tf::StampedTransform& stf = mocap.linkPoses.at({shape.name, pair.first});
			    const tf::StampedTransform& stf = mocap.linkPoses.at({shape.name, "link"});

			    Eigen::Isometry3d T;
			    tf::transformTFToEigen(stf, T);
			    seg.states.push_back({T});

//			    std::cerr << T.matrix() << std::endl;
		    }

	    }
    }

    return 0;
}
