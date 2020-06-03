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

#include "mps_simulation/loading.h"
#include "mps_simulation/paths.h"

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometric_shapes/shape_operations.h>
#include <geometric_shapes/mesh_operations.h>
#include <geometric_shapes/body_operations.h>
#include <geometric_shapes/shape_to_marker.h>

#include "tinyxml.h"

#include <Eigen/Geometry>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;


//#ifndef DEBUG_MESH_LOADING
#define DEBUG_MESH_LOADING true
//#endif

//#ifndef USE_CONVEX_DECOMPOSITION
#define USE_CONVEX_DECOMPOSITION false
//#endif

#if DEBUG_MESH_LOADING
#include <visualization_msgs/MarkerArray.h>
#include <random>
#endif

#if USE_CONVEX_DECOMPOSITION
#include <VHACD.h>
#endif


namespace mps
{


// Pilfered from RViz
double getUnitScaling(const std::string& filename)
{
	float unit_scale = 1.0;
	std::string ext = fs::path(filename).extension().string();
	std::transform(ext.begin(), ext.end(), ext.begin(),
	               [](unsigned char c){ return std::tolower(c); });
	if (ext == ".dae")
	{
		// For some reason, createMeshFromResource doesn't use the <units> from a DAE
		// Use the resource retriever to get the data.
		TiXmlDocument xmlDoc(filename);
		xmlDoc.LoadFile();

		// Find the appropriate element if it exists
		if(!xmlDoc.Error())
		{
			TiXmlElement * colladaXml = xmlDoc.FirstChildElement("COLLADA");
			if(colladaXml)
			{
				TiXmlElement *assetXml = colladaXml->FirstChildElement("asset");
				if(assetXml)
				{
					TiXmlElement *unitXml = assetXml->FirstChildElement("unit");
					if (unitXml && unitXml->Attribute("meter"))
					{
						// Failing to convert leaves unit_scale as the default.
						if(unitXml->QueryFloatAttribute("meter", &unit_scale) != 0)
							ROS_WARN_STREAM("getMeshUnitRescale::Failed to convert unit element meter attribute to determine scaling. unit element: "
								                << *unitXml);
					}
				}
			}
		}
		return unit_scale;
	}
	else
	{
		return 1.0;
	}
}

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

using Pose = Eigen::Isometry3d;
Pose parsePose(TiXmlElement* xPose)
{
	if (!xPose)
	{
		return Pose::Identity();
	}

	if (xPose->Attribute("frame"))
	{
		std::string frame;
		if (xPose->QueryStringAttribute("frame", &frame) == TIXML_SUCCESS)
		{
			if (!frame.empty())
			{
				throw std::runtime_error("This code cannot handle arbitrary frame parents in the SDF.");
			}
		}
	}

	std::cerr << xPose->GetText() << std::endl;
	std::vector<double> posexyz = splitParams(xPose->GetText());

	Pose pose;
	pose.translation() = Eigen::Map<Eigen::Vector3d>(posexyz.data());
	pose.linear() = Eigen::Matrix3d(Eigen::AngleAxisd(posexyz[3], Eigen::Vector3d::UnitZ())
	                                * Eigen::AngleAxisd(posexyz[4], Eigen::Vector3d::UnitY())
	                                * Eigen::AngleAxisd(posexyz[5], Eigen::Vector3d::UnitX()));
	return pose;
}

geometry_msgs::Pose toMsg(const Pose& P)
{
	geometry_msgs::Pose msg;
	msg.position.x = P.translation().x();
	msg.position.y = P.translation().y();
	msg.position.z = P.translation().z();
	Eigen::Quaterniond q(P.linear());
	msg.orientation.x = q.x();
	msg.orientation.y = q.y();
	msg.orientation.z = q.z();
	msg.orientation.w = q.w();

	return msg;
}


std::map<std::string, std::shared_ptr<GazeboModel>> getWorldFileModels(const std::string& gazeboWorldFilename, const std::set<std::string>& modelsToIgnore)
{
	std::map<std::string, std::shared_ptr<GazeboModel>> shapeModels;
	TiXmlDocument doc(gazeboWorldFilename);
	doc.LoadFile();

	#if DEBUG_MESH_LOADING
	std::random_device rd;
	std::mt19937 gen(rd());
	std::uniform_real_distribution<float> uni(0.0, std::nextafter(1.0, std::numeric_limits<float>::max()));
	static ros::NodeHandle pnh("~");
	static ros::Publisher debugPub = pnh.advertise<visualization_msgs::MarkerArray>("debug_shapes", 10, true);
	sleep(3);
	visualization_msgs::MarkerArray debugShapes;
	#endif

	#if USE_CONVEX_DECOMPOSITION
	VHACD::IVHACD* interfaceVHACD = VHACD::CreateVHACD();
	#endif

	TiXmlElement* root = doc.RootElement();
	if (!root)
	{
		ROS_FATAL_STREAM("Unable to open Gazebo world file '" << gazeboWorldFilename << "'. Unable to load object models.");
		return {};
	}

//	TiXmlElement* xSdf = root->FirstChildElement("sdf");
	TiXmlElement* xWorld = root->FirstChildElement("world");
	TiXmlElement* xModState = xWorld->FirstChildElement("state");

	for (TiXmlElement* xModel = xWorld->FirstChildElement("model"); nullptr != xModel;
	     xModel = xModel->NextSiblingElement("model"))
	{
		std::string name(xModel->Attribute("name"));
		std::cerr << name << std::endl;

		auto mod = std::make_shared<GazeboModel>();
		mod->name = name;

		// skip loading e.g. camera and table
		if (modelsToIgnore.find(name) != modelsToIgnore.end())
		{
			continue;
		}

		// Load the model scale from the state
		Eigen::Vector3d modScale = Eigen::Vector3d::Ones();
		for (TiXmlElement* xMods = xModState->FirstChildElement("model"); nullptr != xMods;
		     xMods = xMods->NextSiblingElement("model"))
		{
			if(xMods->Attribute("name") == mod->name)
			{
				std::cerr << "scale: " << xMods->FirstChildElement("scale")->GetText() << std::endl;
				std::vector<double> temp = splitParams(xMods->FirstChildElement("scale")->GetText());
				modScale[0] = temp[0];
				modScale[1] = temp[1];
				modScale[2] = temp[2];
				break;
			}
		}

		Eigen::Isometry3d modelTvisual;
		shapes::ShapePtr shapePtr;
		bodies::BodyPtr bodyPtr;

		// Loop through all links in model
		for (TiXmlElement* xLink = xModel->FirstChildElement("link"); nullptr != xLink;
		     xLink = xLink->NextSiblingElement("link"))
		{
			std::string linkName(xLink->Attribute("name"));

			Eigen::Isometry3d modelTlink = parsePose(xLink->FirstChildElement("pose"));

			TiXmlElement* xVisual = xLink->FirstChildElement("visual");
			if (!xVisual) { continue; }
			Eigen::Isometry3d linkTvisual = parsePose(xVisual->FirstChildElement("pose"));

			TiXmlElement* xGeometry = xVisual->FirstChildElement("geometry");
			if (!xGeometry) { continue; }
			// No offset is allowed in SDF between the visual and the geometry
			// TODO: Geometry.Scale is not handled

			modelTvisual = modelTlink * linkTvisual;
			modelTvisual.translation() = modelTvisual.translation().cwiseProduct(modScale); // Apply model scale

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

				primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = BoxXYZ[0] * modScale[0];
				primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = BoxXYZ[1] * modScale[1];
				primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = BoxXYZ[2] * modScale[2];

				shapePtr = shapes::ShapePtr(shapes::constructShapeFromMsg(primitive));
				bodyPtr = bodies::BodyPtr(bodies::constructBodyFromMsg(primitive, toMsg(modelTvisual)));

			}
			else if (xCylinder)
			{
				std::cerr << "Is Cylinder!" << std::endl;
				shape_msgs::SolidPrimitive primitive;
				primitive.type = shape_msgs::SolidPrimitive::CYLINDER;
				primitive.dimensions.resize(2);
				char* end_ptr;

				primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = std::strtod(xCylinder->FirstChildElement("length")->GetText(), &end_ptr) * modScale[2];
				primitive.dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = std::strtod(xCylinder->FirstChildElement("radius")->GetText(), &end_ptr) * std::max(modScale[0], modScale[1]);

				shapePtr = shapes::ShapePtr(shapes::constructShapeFromMsg(primitive));
				bodyPtr = bodies::BodyPtr(bodies::constructBodyFromMsg(primitive, toMsg(modelTvisual)));

//				shapes::Cylinder* s = dynamic_cast<shapes::Cylinder*>(shapes::constructShapeFromMsg(primitive));
//				s->print(std::cerr);

			}
			else if (xMesh)
			{
				std::cerr << "Is Mesh!" << std::endl;
				std::string pathuri = xMesh->FirstChildElement("uri")->GetText();

				const std::string model_path = parseModelURL(pathuri);
				double unitScale = getUnitScaling(model_path);

				std::cerr << "Scaling: '" << model_path << "': " << unitScale << std::endl;

				shapes::Mesh* m = shapes::createMeshFromResource("file://" + model_path);

				// Apply unit & state scaling
				for (unsigned i = 0; i <  m->vertex_count; ++i)
				{
					m->vertices[3 * i]     *= unitScale * modScale[0];
					m->vertices[3 * i + 1] *= unitScale * modScale[1];
					m->vertices[3 * i + 2] *= unitScale * modScale[2];
				}

				shapePtr = shapes::ShapePtr(m);

//				m->mergeVertices(1e-6);
				m->computeTriangleNormals();
				m->computeVertexNormals();
				std::cerr << "Vertices: " << m->vertex_count << ", Faces: " << m->triangle_count << std::endl;

				#if USE_CONVEX_DECOMPOSITION
				interfaceVHACD->Clean();
				VHACD::IVHACD::Parameters params;
				params.m_minVolumePerCH = 1e-9;
				static_assert(std::is_same<uint32_t*, decltype(shapes::Mesh::triangles)>::value, "Triangle type mismatch");
				interfaceVHACD->Compute(m->vertices, m->vertex_count, m->triangles, m->triangle_count, params);
				for (size_t h = 0; h < interfaceVHACD->GetNConvexHulls(); ++h)
				{
					VHACD::IVHACD::ConvexHull hull;
					interfaceVHACD->GetConvexHull(h, hull);

					// TODO: Re-center about 0

					auto localShape = std::make_shared<shapes::Mesh>();//hull.m_nPoints, hull.m_nTriangles);
					memcpy(localShape->vertices, hull.m_points, sizeof(double) * hull.m_nPoints * 3);
					memcpy(localShape->triangles, hull.m_triangles, sizeof(uint32_t) * hull.m_nTriangles * 3);
					localShape->print(std::cerr);

					localShape->computeTriangleNormals();
					localShape->computeVertexNormals();
					std::cerr << "Vertices: " << localShape->vertex_count << ", Faces: " << localShape->triangle_count << std::endl;

					#if DEBUG_MESH_LOADING
					visualization_msgs::Marker marker;
					shapes::constructMarkerFromShape(localShape.get(), marker, true);
					// Do some more marker stuff here
					marker.color.a = 1.0;
					marker.color.r = uni(gen);
					marker.color.g = uni(gen);
					marker.color.b = uni(gen);
					marker.pose = toMsg(modelTvisual);
					marker.header.frame_id = "mocap_" + name; //"table_surface";
					marker.header.stamp = ros::Time::now();
					marker.ns = name;
					marker.id = 2 + h;
					marker.frame_locked = true;
					debugShapes.markers.push_back(marker);
					#endif
				}
				#endif

				shapes::ShapeMsg mesh;
				constructMsgFromShape(m, mesh);
				bodyPtr = bodies::BodyPtr(bodies::constructBodyFromMsg(mesh, toMsg(modelTvisual)));

				auto* cvx = dynamic_cast<bodies::ConvexMesh*>(bodyPtr.get());
				if (cvx) { std::cerr << "Converted to convex mesh. :(" << std::endl; }

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
//				mod->bodies.insert({linkName, bodies::BodyPtr(bodies::constructBodyFromMsg(plane, toMsg(modelTvisual)))});
//				shapeModels.push_back(mod);
				continue;
			}
			else
			{
				ROS_WARN_STREAM("Unknown shape type: " << name << ".");
				continue;
			}

			#if DEBUG_MESH_LOADING
			visualization_msgs::Marker marker;
			shapes::constructMarkerFromShape(shapePtr.get(), marker, true);
			// Do some more marker stuff here
			marker.color.a = 1.0;
			marker.color.r = uni(gen);
			marker.color.g = uni(gen);
			marker.color.b = uni(gen);
			marker.pose = toMsg(modelTvisual);
			marker.header.frame_id = "mocap_" + name; //"table_surface";
			marker.header.stamp = ros::Time::now();
			marker.ns = name;
			marker.id = 1;
			marker.frame_locked = true;
			debugShapes.markers.push_back(marker);
			#endif

			mod->shapes.insert({linkName, shapePtr});
			mod->bodies.insert({linkName, bodyPtr});
		}

		if (mod->shapes.size() != mod->bodies.size())
		{
			throw std::logic_error("Shape/Body array size mismatch.");
		}

		// Don't load models with no geometry.
		if (!mod->shapes.empty())
			shapeModels.insert({name, mod});
	}
	#if DEBUG_MESH_LOADING
	debugPub.publish(debugShapes);
	std::cerr << "Published debugging." << std::endl;
	#endif
	return shapeModels;
}

}
