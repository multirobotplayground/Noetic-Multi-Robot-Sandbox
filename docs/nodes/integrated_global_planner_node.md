# IntegratedGlobalPlannerNode

Source: [IntegratedGlobalPlannerNode.cpp](../../src/multirobotexploration/source/navigation/IntegratedGlobalPlannerNode.cpp)

## Parameters

* ```id```

Id of this robot.

* ```rate```

Main loop rate in hertz.

* ```queue_size```

Queue size of publishers and subscribers.

* ```reach_threshold```

* ```stuck_time_threshold```

## Subscribed Topics

* ```<namespace>/c_space``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/integrated_global_planner/goal``` ([geometry_msgs::Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))

* ```<namespace>/integrated_global_planner/stop``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

* ```<namespace>/average_velocity``` ([std_msgs::Float32](https://docs.ros.org/en/api/std_msgs/html/msg/Float32.html))

## Published Topics

* ```<namespace>/integrated_global_planner/path``` ([visualization_msgs::Marker](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

* ```<namespace>/integrated_global_planner/finish``` ([std_msgs::String](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

* ```<namespace>/integrated_global_planner/current_path``` ([nav_msgs::Path](https://docs.ros.org/en/api/nav_msgs/html/msg/Path.html))

<!-- ## Published Transforms

* ```odom``` -->
