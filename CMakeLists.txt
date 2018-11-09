cmake_minimum_required(VERSION 2.8.3)
project(mps_voxels)

## Compile as C++14
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  depth_image_proc
  image_geometry
  image_transport
  message_generation
  mps_msgs
  moveit_ros_planning
  octomap_msgs
#  octomap_ros
#  octomap_server
  pcl_conversions
  pcl_ros
  realtime_tools
  roscpp
  rospy
  sensor_msgs
  tf
  tf_conversions

  victor_hardware_interface
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

find_package(CUDA REQUIRED)
add_subdirectory(CudaSift)
include_directories(SYSTEM CudaSift/include)
link_libraries(cudaSift)

find_package(OCTOMAP 1.9 REQUIRED)
include_directories(SYSTEM ${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

find_package(OpenCV 3.4 REQUIRED COMPONENTS
  cudafeatures2d
  cudafilters
  cudaoptflow
  xfeatures2d
)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIR})
link_libraries(${OpenCV_LIBS})

include_directories(SYSTEM COB/src/misc)
include_directories(SYSTEM spectra/include)

# CGAL and its components
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE)
find_package(CGAL 4.12 REQUIRED COMPONENTS
  Polygon_mesh_processing
  QP_solver
)
include( ${CGAL_USE_FILE} )
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Force CGAL to maintain CMAKE flags")

option(ASAN "Adds flags for AddressSanitizer." OFF)
if(ASAN)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
  set(CMAKE_CXX_FLAGS_DEBUG "-fsanitize=address -fno-omit-frame-pointer ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_LINKER_FLAGS_DEBUG "-fsanitize=address -fno-omit-frame-pointer ${CMAKE_STATIC_LINKER_FLAGS_DEBUG}")
endif()

option(CLION "Adds flags for debugging in clion." OFF)
if(CLION)
  set(CMAKE_BUILD_TYPE "Debug")
  set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")
  set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0")
endif()
set(CMAKE_BUILD_TYPE RelWithDebInfo)
#set(CMAKE_BUILD_TYPE Debug)

macro(use_strict_compile_flags)
  set(BUILD_FLAGS "-Wshadow -Wall -Wextra -Wpedantic -Werror=return-type")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${BUILD_FLAGS}")
endmacro()

use_strict_compile_flags()

macro(use_openmp)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
  set(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} -lgomp")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -lgomp")
endmacro()

use_openmp()

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a exec_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a exec_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   CompleteShape.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

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
#  LIBRARIES mps_voxels
  CATKIN_DEPENDS
    cv_bridge
    depth_image_proc
    image_geometry
    image_transport
    message_runtime
    moveit_ros_planning
    mps_msgs
    octomap_msgs
#    octomap_ros
#    octomap_server
    pcl_conversions
    pcl_ros
    roscpp
    rospy
    sensor_msgs
    tf
    tf_conversions
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

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/octree_utils.cpp
  src/${PROJECT_NAME}/pointcloud_utils.cpp
  src/${PROJECT_NAME}/image_utils.cpp
  src/${PROJECT_NAME}/segmentation_utils.cpp
  src/${PROJECT_NAME}/shape_utils.cpp
  src/${PROJECT_NAME}/colormap.cpp
  src/${PROJECT_NAME}/Ultrametric.cpp
  src/${PROJECT_NAME}/Manipulator.cpp
  src/${PROJECT_NAME}/VictorManipulator.cpp
  src/${PROJECT_NAME}/MotionModel.cpp
  src/${PROJECT_NAME}/ROI.cpp
  src/${PROJECT_NAME}/Tracker.cpp
  src/${PROJECT_NAME}/CudaTracker.cpp
  src/${PROJECT_NAME}/TargetDetector.cpp
  src/${PROJECT_NAME}/graph_matrix_utils.cpp
  src/${PROJECT_NAME}/LocalOctreeServer.cpp
  src/${PROJECT_NAME}/planning/State.cpp
  src/${PROJECT_NAME}/planning/Action.cpp
  src/${PROJECT_NAME}/planning/MotionPlanner.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_shape_completion_manager src/shape_completion_manager.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_shape_completion_manager PROPERTIES OUTPUT_NAME shape_completion_manager PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_shape_completion_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_shape_completion_manager
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_entropy src/entropy.cpp)
set_target_properties(${PROJECT_NAME}_entropy PROPERTIES OUTPUT_NAME entropy PREFIX "")
add_dependencies(${PROJECT_NAME}_entropy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_entropy
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


cuda_add_executable(${PROJECT_NAME}_tracker src/tracker.cpp)
set_target_properties(${PROJECT_NAME}_tracker PROPERTIES OUTPUT_NAME tracker PREFIX "")
add_dependencies(${PROJECT_NAME}_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_tracker
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_test_graph_segmentation src/test_graph_segmentation.cpp)
set_target_properties(${PROJECT_NAME}_test_graph_segmentation PROPERTIES OUTPUT_NAME test_graph_segmentation PREFIX "")
add_dependencies(${PROJECT_NAME}_test_graph_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_graph_segmentation
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_explorer src/explorer.cpp)
set_target_properties(${PROJECT_NAME}_explorer PROPERTIES OUTPUT_NAME explorer PREFIX "")
add_dependencies(${PROJECT_NAME}_explorer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_explorer
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Notify any IDE that headers are part of project
file(GLOB_RECURSE IncludeFiles "include/*")
add_custom_target(${PROJECT_NAME}_headers SOURCES ${IncludeFiles})


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
install(PROGRAMS
  scripts/spectral_graph_clustering.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mps_voxels.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
