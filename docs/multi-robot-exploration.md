# Multi-robot Exploration

It contains a multi-robot exploration stack and an implementation and integration of this [research](https://arxiv.org/abs/2309.13494) that allow you to start your development out-of-the-box.

## Nodes

  - [Policies](../docs/multi-robot-exploration.md)
    - [Yamauchi1999 node](../docs/nodes/yamauchi1999_node.md)
    - [RandomizedSocialWelfare node](../docs/nodes/randomized_social_welfare_node.md)
    - [Alysson2024 node](../docs/nodes/alysson2024_node.md)
  - [Navigation](../docs/multi-robot-exploration.md)
    - [LocalPlannerNode](../docs/nodes/local_planner_node.md)
    - [IntegratedGlobalPlannerNode](../docs/nodes/integrated_global_planner_node.md)
  - [Mapping](../docs/multi-robot-exploration.md)
    - [CSpaceNode](../docs/nodes/cspace_node.md)
    - [MapStitchingNode](../docs/nodes/map_stitching_node.md)
    - [LocalCSpaceNode](../docs/nodes/local_cspace_node.md)
  - [Localization](../docs/multi-robot-exploration.md)
    - [AverageVelocityEstimatorNode](../docs/nodes/average_velocity_node.md)
    - [GmappingPoseNode](../docs/nodes/gmapping_pose_node.md)
    - [RelativePoseEstimatorNode](../docs/nodes/relative_pose_estimator_node.md)
  - [Lidar](../docs/multi-robot-exploration.md)
    - [LaserToWorldNode](../docs/nodes/laser_to_world_node.md)
  - [Frontiers](../docs/multi-robot-exploration.md)
    - [FrontierDiscoveryNode](../docs/nodes/frontier_discovery_node.md)
  - [Communication](../docs/multi-robot-exploration.md)
    - [MockCommunicationModelNode](../docs/nodes/mock_communication_model_node.md)
  - [Objects and Algorithms](../docs/multi-robot-exploration.md)
    - [RendezvosPlan](../src/multirobotexploration/source/common/RendezvousPlan.cpp)
    - [SearchAlgorithms](../src/multirobotexploration/source/common/SearchAlgorithms.cpp)
    - [Common](../src/multirobotexploration/include/common/Common.h)