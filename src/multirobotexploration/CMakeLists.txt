cmake_minimum_required(VERSION 3.0.2)
project(multirobotexploration)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  roslib
  message_generation
  message_runtime
  dynamic_reconfigure
  teb_local_planner
  multirobotsimulations
)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS roscpp
    std_msgs
    sensor_msgs
    nav_msgs
    tf2_ros
    roslib
    message_generation
    message_runtime
    dynamic_reconfigure
    teb_local_planner
    multirobotsimulations
)


###########
## Build ##
###########

include_directories(
  include/common
  include/nodes
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_library(
  ${PROJECT_NAME}
  source/common/SearchAlgorithms.cpp
  source/common/RendezvousPlan.cpp
)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(DENABLE_PRECOMPILED_HEADERS OFF)

###################
## communication ##
###################

add_executable(mockcommunicationmodelnode source/communication/MockCommunicationModelNode.cpp)
target_link_libraries(mockcommunicationmodelnode multirobotexploration ${catkin_LIBRARIES})

##################
## localization ##
##################
add_executable(gmappingposenode source/localization/GmappingPoseNode.cpp)
target_link_libraries(gmappingposenode ${catkin_LIBRARIES})

add_executable(averagevelocityestimatornode source/localization/AverageVelocityEstimatorNode.cpp)
target_link_libraries(averagevelocityestimatornode multirobotexploration ${catkin_LIBRARIES})

add_executable(relativeposeestimatornode source/localization/RelativePoseEstimatorNode.cpp)
target_link_libraries(relativeposeestimatornode multirobotexploration ${catkin_LIBRARIES})

add_executable(perfect_odometry_gazebo source/localization/GazeboPerfectOdometry.cpp)
target_link_libraries(perfect_odometry_gazebo multirobotexploration ${catkin_LIBRARIES})

################
## navigation ##
################
add_executable(integratedglobalplannernode source/navigation/IntegratedGlobalPlannerNode.cpp)
target_link_libraries(integratedglobalplannernode multirobotexploration ${catkin_LIBRARIES})

add_executable(localplannernode source/navigation/LocalPlannerNode.cpp)
target_link_libraries(localplannernode multirobotexploration ${catkin_LIBRARIES})

###############
## frontiers ##
###############
add_executable(frontierdiscoverynode source/frontier/FrontierDiscoveryNode.cpp)
target_link_libraries(frontierdiscoverynode multirobotexploration ${catkin_LIBRARIES})

#############
## mapping ##
#############
add_executable(cspacenode source/map/CSpaceNode.cpp)
target_link_libraries(cspacenode multirobotexploration ${catkin_LIBRARIES})

add_executable(localcspacenode source/map/LocalCSpaceNode.cpp)
target_link_libraries(localcspacenode multirobotexploration ${catkin_LIBRARIES})

add_executable(mapstitchingnode source/map/MapStitchingNode.cpp)
target_link_libraries(mapstitchingnode multirobotexploration ${OpenCV_LIBS} ${catkin_LIBRARIES})

add_executable(global_map_stitching_node source/map/GlobalMapStitchingNode.cpp)
target_link_libraries(global_map_stitching_node multirobotexploration ${catkin_LIBRARIES})

add_executable(maprequestservicenode source/map/MapRequestServiceNode.cpp)
target_link_libraries(maprequestservicenode multirobotexploration ${catkin_LIBRARIES})

#############
## sensors ##
#############
add_executable(lasertoworldnode source/laser/LaserToWorldNode.cpp)
target_link_libraries(lasertoworldnode multirobotexploration ${catkin_LIBRARIES})

##############
## policies ##
##############
add_executable(alysson2025 source/policies/Alysson2025Node.cpp)
target_link_libraries(alysson2025 multirobotexploration ${catkin_LIBRARIES})

add_executable(alysson2024 source/policies/Alysson2024Node.cpp)
target_link_libraries(alysson2024 multirobotexploration ${catkin_LIBRARIES})

add_executable(priorityallocationnode source/policies/PriorityAllocatioNode.cpp)
target_link_libraries(priorityallocationnode multirobotexploration ${catkin_LIBRARIES})

add_executable(randomizedsocialwelfare source/policies/RandomizedSocialWelfareNode.cpp)
target_link_libraries(randomizedsocialwelfare multirobotexploration ${catkin_LIBRARIES})

add_executable(yamauchi1999 source/policies/Yamauchi1999Node.cpp)
target_link_libraries(yamauchi1999 multirobotexploration ${catkin_LIBRARIES})


#############
## Install ##
#############
catkin_install_python(PROGRAMS source/data_analysis/explored_area.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)