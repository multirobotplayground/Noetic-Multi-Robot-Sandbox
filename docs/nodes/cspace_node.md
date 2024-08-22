# CSpaceNode

Source: [CSpaceNode.cpp](../../src/multirobotexploration/source/map/CSpaceNode.cpp)

## Parameters

* ```id```

* ```rate```

* ```queue_size```

* ```max_lidar_range```

* ```free_inflation_radius```

* ```ocu_inflation_radius```

* ```lidar_sources```

## Subscribed Topics

* ```<namespace>/map``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```<namespace>/laser_to_world/lidar_occ_<i>``` ([geometry_msgs::PoseArray](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))

* ```/robot_<id>/local_planner/optimal_poses``` ([geometry_msgs::PoseArray](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))

## Published Topics

* ```<namespace>/c_space``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

<!-- ## Published Transforms

* ```odom``` -->
