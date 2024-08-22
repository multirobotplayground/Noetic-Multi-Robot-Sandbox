# MapStitchingNode

Source: [MapStitchingNode.cpp](../../src/multirobotexploration/source/map/MapStitchingNode.cpp)

## Parameters

* ```/robots```

* ```id```

* ```queue_size```

* ```rate```

## Subscribed Topics

* ```<namespace>/relative_pose_estimator/relative_start``` ([geometry_msgs::PoseArray](https://docs.ros.org/en/api/geometry_msgs/html/msg/PoseArray.html))

* ```<namespace>/map``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

* ```<namespace>/mock_communication_model/robots_in_comm``` ([std_msgs::Int8MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Int8MultiArray.html))

* ```<namespace>/robot_<i>/fusion``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

## Published Topics

* ```<namespace>/fusion``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

<!-- ## Published Transforms

* ```odom``` -->
