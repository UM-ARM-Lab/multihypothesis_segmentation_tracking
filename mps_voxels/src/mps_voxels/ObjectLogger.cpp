//
// Created by arprice on 1/31/19.
//

#include "mps_voxels/ObjectLogger.h"

#include <pcl/io/file_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/impl/pcd_io.hpp>

#include <assimp/Exporter.hpp>
#include <assimp/scene.h>

namespace mps
{

bool ObjectLogger::logObject(const Object* obj, const std::string& location, const std::string& name)
{
	obj->occupancy->write(location + "/" + name + ".ot");
//	pcl::io::save(location + "/" + name + ".pcd", *obj->segment);

//	Assimp::Exporter ex;
//	aiScene s;
//	aiMesh m;
//	s.mNumMeshes = 1;
//	s.mMeshes;

//	obj->approximation->
	return true;
}

}
