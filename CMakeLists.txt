cmake_minimum_required(VERSION 2.8.3)
project(mps_voxels)

## Compile as C++14
add_compile_options(-std=c++14)

## No, really. Use C++14 (If a prior package sets c++11, sometimes that will be used instead of 14)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  arm_video_recorder
  cv_bridge
  depth_image_proc
  image_geometry
  image_transport
  message_generation
  mps_msgs
  mps_shape_completion_msgs
  moveit_ros_planning
  octomap_msgs
#  octomap_ros
#  octomap_server
  pcl_conversions
  pcl_ros
  realtime_tools
  rosbag
  roscpp
  rospy
  sensor_msgs
  tf
  tf_conversions

  victor_hardware_interface
)

## System dependencies are found with CMake's conventions

# CGAL and its components
# NB: CGAL likes to clobber global variables, e.g. Boost_LIBRARIES, so make sure you include it first
set(CGAL_DO_NOT_WARN_ABOUT_CMAKE_BUILD_TYPE TRUE CACHE BOOL "Disable CGAL complaints about performance flags.")
set(CGAL_DONT_OVERRIDE_CMAKE_FLAGS TRUE CACHE BOOL "Force CGAL to maintain CMAKE flags")
find_package(CGAL 4.12 REQUIRED COMPONENTS
  Polygon_mesh_processing
  QP_solver
)
include( ${CGAL_USE_FILE} )

find_package(Boost REQUIRED COMPONENTS
  filesystem
)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

find_package(CUDA REQUIRED)
if (CMAKE_COMPILER_IS_GNUCC AND (CMAKE_CXX_COMPILER_VERSION VERSION_LESS 7 AND CUDA_VERSION VERSION_LESS 10) OR CUDA_VERSION VERSION_GREATER_EQUAL 10)
  add_subdirectory(CudaSift)
  include_directories(SYSTEM CudaSift/include)
  link_libraries(cudaSift)
  add_definitions(-DHAS_CUDA_SIFT)
endif()

find_package(OCTOMAP REQUIRED)
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

find_package(PCL REQUIRED COMPONENTS
  common
  io
  filters
)
include_directories(SYSTEM ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(SYSTEM COB/src/misc)

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
#set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DCGAL_DISABLE_ROUNDING_MATH_CHECK")
#set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -DCGAL_NO_ASSERTIONS")

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
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    arm_video_recorder
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
    rosbag
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
  src/${PROJECT_NAME}/AABB.cpp
  src/${PROJECT_NAME}/JaccardMatch.cpp
  src/${PROJECT_NAME}/Ultrametric.cpp
  src/${PROJECT_NAME}/ValueTree.cpp
  src/${PROJECT_NAME}/SceneCut.cpp
  src/${PROJECT_NAME}/MCMCTreeCut.cpp
  src/${PROJECT_NAME}/SegmentationTreeSampler.cpp
  src/${PROJECT_NAME}/image_output.cpp
  src/${PROJECT_NAME}/Manipulator.cpp
  src/${PROJECT_NAME}/VictorManipulator.cpp
  src/${PROJECT_NAME}/Object.cpp
  src/${PROJECT_NAME}/OccupancyData.cpp
  src/${PROJECT_NAME}/MotionModel.cpp
  src/${PROJECT_NAME}/OctreeMotionModel.cpp
  src/${PROJECT_NAME}/ObjectActionModel.cpp
  src/${PROJECT_NAME}/Particle.cpp
  src/${PROJECT_NAME}/ParticleFilter.cpp
  src/${PROJECT_NAME}/ROI.cpp
  src/${PROJECT_NAME}/SensorHistorian.cpp
  src/${PROJECT_NAME}/Tracker.cpp
  src/${PROJECT_NAME}/CudaTracker.cpp
  src/${PROJECT_NAME}/SiamTracker.cpp
  src/${PROJECT_NAME}/TargetDetector.cpp
  src/${PROJECT_NAME}/graph_matrix_utils.cpp
  src/${PROJECT_NAME}/LocalOctreeServer.cpp
  src/${PROJECT_NAME}/VoxelRegion.cpp
  src/${PROJECT_NAME}/Scene.cpp
  src/${PROJECT_NAME}/util/package_paths.cpp
  src/${PROJECT_NAME}/ObjectLogger.cpp
  src/${PROJECT_NAME}/logging/DataLog.cpp
  src/${PROJECT_NAME}/logging/log_cv_mat.cpp
  src/${PROJECT_NAME}/logging/log_cv_roi.cpp
  src/${PROJECT_NAME}/logging/log_segmentation_info.cpp
  src/${PROJECT_NAME}/logging/log_sensor_history.cpp
  src/${PROJECT_NAME}/video_graph.cpp
  src/${PROJECT_NAME}/planning/State.cpp
  src/${PROJECT_NAME}/planning/Action.cpp
  src/${PROJECT_NAME}/planning/MotionPlanner.cpp
  )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${PCL_IO_LIBRARIES}
  ${Boost_LIBRARIES}
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


#cuda_add_executable(${PROJECT_NAME}_tracker src/tracker.cpp)
#set_target_properties(${PROJECT_NAME}_tracker PROPERTIES OUTPUT_NAME tracker PREFIX "")
#add_dependencies(${PROJECT_NAME}_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_tracker
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

#add_executable(${PROJECT_NAME}_test_graph_segmentation src/test_graph_segmentation.cpp)
#set_target_properties(${PROJECT_NAME}_test_graph_segmentation PROPERTIES OUTPUT_NAME test_graph_segmentation PREFIX "")
#add_dependencies(${PROJECT_NAME}_test_graph_segmentation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_test_graph_segmentation
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)


cuda_add_executable(${PROJECT_NAME}_explorer src/explorer.cpp)
set_target_properties(${PROJECT_NAME}_explorer PROPERTIES OUTPUT_NAME explorer PREFIX "")
add_dependencies(${PROJECT_NAME}_explorer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_explorer
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

cuda_add_executable(${PROJECT_NAME}_test_segmentation_sampling src/test_segmentation_sampling.cpp)
set_target_properties(${PROJECT_NAME}_test_segmentation_sampling PROPERTIES OUTPUT_NAME test_segmentation_sampling PREFIX "")
add_dependencies(${PROJECT_NAME}_test_segmentation_sampling ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_segmentation_sampling
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

#add_executable(${PROJECT_NAME}_test_object_action_model src/test_object_action_model.cpp)
#set_target_properties(${PROJECT_NAME}_test_object_action_model PROPERTIES OUTPUT_NAME test_object_action_model PREFIX "")
#add_dependencies(${PROJECT_NAME}_test_object_action_model ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_test_object_action_model
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

#add_executable(${PROJECT_NAME}_test_icp src/test_icp.cpp)
#set_target_properties(${PROJECT_NAME}_test_icp PROPERTIES OUTPUT_NAME test_icp PREFIX "")
#add_dependencies(${PROJECT_NAME}_test_icp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#target_link_libraries(${PROJECT_NAME}_test_icp
#  ${PROJECT_NAME}
#  ${catkin_LIBRARIES}
#)

add_executable(${PROJECT_NAME}_test_particle_filter src/test_particle_filter.cpp)
set_target_properties(${PROJECT_NAME}_test_particle_filter PROPERTIES OUTPUT_NAME test_particle_filter PREFIX "")
add_dependencies(${PROJECT_NAME}_test_particle_filter ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_test_particle_filter
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
  scripts/matlab_action_wrapper.py
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

file(GLOB unit_tests_source "test/*.cpp")
list(SORT unit_tests_source)
foreach(utest_src_file ${unit_tests_source})
  get_filename_component(test_base ${utest_src_file} NAME_WE)

  catkin_add_gtest(${PROJECT_NAME}_${test_base} ${utest_src_file})
  #  add_executable(${test_base} ${utest_src_file})

  target_link_libraries(${PROJECT_NAME}_${test_base}
    ${PROJECT_NAME}
    ${GTEST_LIBRARIES}
    )
endforeach(utest_src_file)
