cmake_minimum_required(VERSION 2.8.3)
project(mps_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  mps_simulation
  mps_voxels
  rviz_cinematographer_msgs
)

include(${mps_voxels_SOURCE_PREFIX}/cmake/mps_macros.cmake)
#include(${mps_voxels_DIR}/mps_macros.cmake)

use_strict_compile_flags()

use_openmp()

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS
  filesystem
)
include_directories(SYSTEM ${Boost_INCLUDE_DIRS})
link_libraries(${Boost_LIBRARIES})


option(GVDB "Use GVDB voxelization." OFF)
if(GVDB)
  #find_package(CUDA 10.2 REQUIRED)
  #include_directories(SYSTEM CUDA_INCLUDE_DIRS)
  find_package(CUDAToolkit 10.2 REQUIRED)
  find_package(GVDB REQUIRED)
endif()


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
#  LIBRARIES mps_test
#  CATKIN_DEPENDS mps_simulation mps_voxels
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
  src/${PROJECT_NAME}/CPUPixelizer.cpp
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

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_generate_gazebo_benchmark src/generate_gazebo_benchmark.cpp)
set_target_properties(${PROJECT_NAME}_generate_gazebo_benchmark PROPERTIES OUTPUT_NAME generate_gazebo_benchmark PREFIX "")
add_dependencies(${PROJECT_NAME}_generate_gazebo_benchmark ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_generate_gazebo_benchmark
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_gazebo_segmentation_gt src/gazebo_segmentation_gt.cpp)
set_target_properties(${PROJECT_NAME}_gazebo_segmentation_gt PROPERTIES OUTPUT_NAME gazebo_segmentation_gt PREFIX "")
add_dependencies(${PROJECT_NAME}_gazebo_segmentation_gt ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_gazebo_segmentation_gt
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  )
if (GVDB)
  target_compile_definitions(${PROJECT_NAME}_gazebo_segmentation_gt PRIVATE VOXELIZER=1)
else()
  target_compile_definitions(${PROJECT_NAME}_gazebo_segmentation_gt PRIVATE VOXELIZER=0)
endif()

add_executable(${PROJECT_NAME}_occupancy_evaluation src/occupancy_evaluation.cpp)
set_target_properties(${PROJECT_NAME}_occupancy_evaluation PROPERTIES OUTPUT_NAME occupancy_evaluation PREFIX "")
add_dependencies(${PROJECT_NAME}_occupancy_evaluation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_occupancy_evaluation
  ${PROJECT_NAME}
)

add_executable(${PROJECT_NAME}_segmentation_evaluation src/segmentation_evaluation.cpp)
set_target_properties(${PROJECT_NAME}_segmentation_evaluation PROPERTIES OUTPUT_NAME segmentation_evaluation PREFIX "")
add_dependencies(${PROJECT_NAME}_segmentation_evaluation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_segmentation_evaluation
  ${PROJECT_NAME}
)

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

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_mps_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
