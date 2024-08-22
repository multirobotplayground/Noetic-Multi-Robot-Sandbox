# LocalPlannerNode

Source: [LocalPlannerNode.cpp](../../src/multirobotexploration/source/navigation/LocalPlannerNode.cpp)

## Parameters

* ```/robots```

* ```rate```

* ```controls_to_share```

* ```waypoints_to_use```

* ```via_points_increment```

* ```use_priority_stop_behavior```

## Subscribed Topics

* ```<namespace>/costmap_converter/obstacles``` ([costmap_converter::ObstacleArrayMsg])

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```<namespace>/sub_goal_nav/current_path``` ([nav_msgs::Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html))

* ```<namespace>/mock_communication_model/robots_in_comm``` ([std_msgs::Int8MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Int8MultiArray.html))
  
## Published Topics

* ```<namespace>/cmd_vel``` ([geometry_msgs::Twist](https://docs.ros.org/en/api/geometry_msgs/html/msg/Twist.html))

* ```<namespace>/local_planner/optimal_poses``` ([geometry_msgs::PoseArray](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))

* ```<namespace>/mre_local_planner/global_via_points``` ([visualization_msgs::Marker](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

<!-- ## Published Transforms

* ```odom``` -->
