cmake_minimum_required(VERSION 3.0.2)
project(multirobotsimulations)

## Compile as C++11  supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  roslib
  message_generation
  message_runtime
)

add_message_files(
   FILES
   CustomImage.msg
   CustomPose.msg
   CustomOcc.msg
   Frontiers.msg
   rendezvous.msg
   MockPackage.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS roscpp
    rospy
    std_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
    roslib
    message_generation
    message_runtime
)

###########
## Build ##
###########

include_directories(
  include 
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)