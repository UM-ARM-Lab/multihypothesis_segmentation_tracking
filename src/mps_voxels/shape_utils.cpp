//
// Created by arprice on 8/31/18.
//

#include "mps_voxels/shape_utils.h"
#include "mps_voxels/shape_utils-impl.hpp"

#include <octomap/octomap_types.h>

template std::shared_ptr<shapes::Mesh> convex_hull<octomap::point3d_collection, octomap::point3d>(const octomap::point3d_collection&);
template std::shared_ptr<shapes::Mesh> prism<octomap::point3d_collection, octomap::point3d>(const octomap::point3d_collection&);
template std::shared_ptr<shapes::Mesh> ZAMBB<octomap::point3d_collection, octomap::point3d>(const octomap::point3d_collection&);
