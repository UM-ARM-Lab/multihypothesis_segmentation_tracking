cmake_minimum_required(VERSION 3.8)
project(mps_simulation)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  gazebo_dev
  gazebo_msgs
  image_geometry
  image_transport
  moveit_core
  mps_msgs
  mps_voxels
  roscpp
  roslib
  tf_conversions
)

if ("-std=c++11" IN_LIST GAZEBO_CXX_FLAGS)
  message(STATUS "Package 'gazebo_dev' has injected c++11 into the CXX_FLAGS. Removing.")
  string(REPLACE "-std=c++11" "" CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS})
endif()

include(${mps_voxels_SOURCE_DIR}/cmake/mps_macros.cmake)
#include(${mps_voxels_DIR}/mps_macros.cmake)

use_strict_compile_flags()

use_openmp()

## System dependencies are found with CMake's conventions

# CGAL and its components
# NB: CGAL likes to clobber global variables, e.g. Boost_LIBRARIES, so make sure you include it first
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE CACHE BOOL "Disable CGAL complaints about performance flags.")
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Force CGAL to maintain CMAKE flags")
find_package(CGAL 4.12 REQUIRED COMPONENTS
  Polygon_mesh_processing
)
include( ${CGAL_USE_FILE} )
# find_package(Boost REQUIRED COMPONENTS system)

find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})

option(GVDB "Use GVDB voxelization." OFF)
if(GVDB)
  #find_package(CUDA 10.2 REQUIRED)
  #include_directories(SYSTEM CUDA_INCLUDE_DIRS)
  find_package(CUDAToolkit 10.2 REQUIRED)
  find_package(GVDB REQUIRED)
endif()

#find_package(vhacd REQUIRED)
#include_directories(SYSTEM ${})

#find_package(yaml-cpp REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
#message(WARNING "YAML-CPP: ${YAML_CPP_INCLUDE_DIR}")
include_directories(SYSTEM ${YAML_CPP_INCLUDE_DIR})
link_libraries(${YAML_CPP_LIBRARIES})

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a exec_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  #  INCLUDE_DIRS include
  #  LIBRARIES mps_simulation
  #  CATKIN_DEPENDS moveit_core
  #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
)

include_directories(SYSTEM
  ${catkin_INCLUDE_DIRS}
)

if (GVDB)
  set(OTHER_SRCS
    src/${PROJECT_NAME}/GVDBVoxelizer.cpp
  )
else()
  set(OTHER_SRCS "")
endif()

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/paths.cpp
  src/${PROJECT_NAME}/loading.cpp
  src/${PROJECT_NAME}/GazeboModel.cpp
  src/${PROJECT_NAME}/GazeboMocap.cpp
  src/${PROJECT_NAME}/ROSVoxelizer.cpp
  ${OTHER_SRCS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
if (GVDB)
  target_link_libraries(${PROJECT_NAME}
  #  ::vhacd
    CUDA::cudart
    GVDB::gvdb
  #  GVDB::gvdbPTX
  )
endif()

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_gazebo_mocap test/system/gazebo_mocap.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_gazebo_mocap PROPERTIES OUTPUT_NAME gazebo_mocap PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_gazebo_mocap ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_gazebo_mocap
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_gazebo_segmentation_gt test/system/gazebo_segmentation_gt.cpp)
set_target_properties(${PROJECT_NAME}_gazebo_segmentation_gt PROPERTIES OUTPUT_NAME gazebo_segmentation_gt PREFIX "")
add_dependencies(${PROJECT_NAME}_gazebo_segmentation_gt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_gazebo_segmentation_gt
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${YAML_CPP_LIBRARIES} # NB: This is not being forwarded correctly from mps_voxels
)
if (GVDB)
  target_compile_definitions(${PROJECT_NAME}_gazebo_segmentation_gt PRIVATE VOXELIZER=1)
else()
  target_compile_definitions(${PROJECT_NAME}_gazebo_segmentation_gt PRIVATE VOXELIZER=0)
endif()

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mps_simulation.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
