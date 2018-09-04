//
// Created by arprice on 8/31/18.
//

#ifndef PROJECT_SHAPE_UTILS_H
#define PROJECT_SHAPE_UTILS_H

//#include <geometric_shapes/bodies.h>
#include <geometric_shapes/shapes.h>

template <typename PointContainerT, typename PointT = typename PointContainerT::value_type>
std::shared_ptr<shapes::Mesh> convex_hull(const PointContainerT& points);

template <typename PointContainerT, typename PointT = typename PointContainerT::value_type>
std::shared_ptr<shapes::Mesh> prism(const PointContainerT& points);

template <typename PointContainerT, typename PointT = typename PointContainerT::value_type>
std::shared_ptr<shapes::Mesh> ZABB(const PointContainerT& points);

#endif // PROJECT_SHAPE_UTILS_H
