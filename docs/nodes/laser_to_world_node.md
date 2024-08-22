# LaserToWorldNode

Source: [LaserToWorldNode.cpp](../../src/multirobotexploration/source/laser/LaserToWorldNode.cpp)

## Parameters

* ```rate```

* ```queue_size```

* ```x```

* ```y```

* ```z```

* ```r```

* ```p```

* ```y```


## Subscribed Topics

* ```<namespace>/scan``` ([sensor_msgs::LaserScan](https://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html))

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```<namespace>/map``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

## Published Topics

* ```<namespace>/laser_to_world/laser_world``` ([geometry_msgs::PoseArray](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))

* ```<namespace>/laser_to_world/laser_occ``` ([geometry_msgs::PoseArray](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))

<!-- ## Published Transforms

* ```odom``` -->
