cmake_minimum_required(VERSION 2.8.3)
project(mps_msgs)

set(CATKIN_PACKAGES
    actionlib_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
)
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS ${CATKIN_PACKAGES} message_generation)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    IndexMap.msg
    ColorMap.msg
    SparseMatrix.msg
    FlowVector.msg
    AABBox2d.msg
    VoxelRegion.msg
    VoxelSegmentation.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
    SegmentGraph.srv
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
    ClusterRigidMotions.action
    MatchFrameSegments.action
    SegmentRGBD.action
    TrackBBox.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
)

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS
    LIBRARIES
    CATKIN_DEPENDS  ${CATKIN_PACKAGES}
                    message_runtime
)

###########
## Build ##
###########


