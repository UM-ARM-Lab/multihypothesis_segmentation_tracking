//
// Created by kunhuang on 6/3/19.
//

#ifndef MPS_MOVEIT_POSE_TYPE_H
#define MPS_MOVEIT_POSE_TYPE_H

#include <moveit/version.h>
#include <Eigen/Geometry>

#define MOVEIT_VERSION_AT_LEAST(x,y,z) (MOVEIT_VERSION_MAJOR>x || (MOVEIT_VERSION_MAJOR>=x && \
                                       (MOVEIT_VERSION_MINOR>y || (MOVEIT_VERSION_MINOR>=y && \
                                                                   MOVEIT_VERSION_PATCH>=z))))

namespace moveit
{
#if MOVEIT_VERSION_AT_LEAST(1, 0, 1)
    using Pose = Eigen::Isometry3d;
#else
    using Pose = Eigen::Affine3d;
#endif
}

#endif //MPS_MOVEIT_POSE_TYPE_H
