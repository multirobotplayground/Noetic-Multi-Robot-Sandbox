# RelativePoseEstimatorNode

Source: [RelativePoseEstimatorNode.cpp](../../src/multirobotexploration/source/localization/RelativePoseEstimatorNode.cpp)

## Parameters

* ```/robots```
  
* ```id```
  
* ```queue_size```
  
* ```rate```

* ```/start_pose_robot_<id>```

## Subscribed Topics

* ```<namespace>/robot_<id>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```<namespace>/mock_communication_model/robots_in_comm``` ([std_msgs::Int8MultiArray](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

## Published Topics

* ```<namespace>/relative_pose_estimator/relative_start``` ([geometry_msgs::PoseArray](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))

* ```<namespace>/relative_pose_estimator/relative_poses``` ([geometry_msgs::PoseArray](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))

* ```<namespace>/relative_pose_estimator/distances``` ([std_msgs::Float64MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Float64MultiArray.html))

* ```<namespace>/relative_pose_estimator/pose_markers``` ([visualization_msgs::Marker](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

* ```<namespace>/relative_pose_estimator/pose_far_markers``` ([visualization_msgs::Marker](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

<!-- ## Published Transforms

* ```odom``` -->
