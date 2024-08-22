# FrontierDiscoveryNode

Source: [FrontierDiscoveryNode.cpp](../../src/multirobotexploration/source/frontier/FrontierDiscoveryNode.cpp)

## Parameters

* ```id```

* ```rate```

* ```queue_size```

* ```max_lidar_range```

## Subscribed Topics

* ```<namespace>/c_space``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```<namespace>/frontier_discovery/compute``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

## Published Topics

* ```<namespace>/frontier_discovery/frontiers_clusters_markers``` ([visualization_msgs::Marker](https://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html))

* ```<namespace>/frontier_discovery/frontiers``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/frontier_discovery/frontiers_clusters``` ([multirobotsimulations::Frontiers](../../src/multirobotsimulations/msg/Frontiers.msg))

<!-- ## Published Transforms

* ```odom``` -->
