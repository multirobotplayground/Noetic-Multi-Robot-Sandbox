# MockCommunicationModelNode

Source: [MockCommunicationModelNode.cpp](../../src/multirobotexploration/source/communication/MockCommunicationModelNode.cpp)

## Parameters

* ```/robots```

* ```id```

* ```queue_size```

* ```comm_dist```

* ```rate```

* ```/start_pose_robot_<id>```

## Subscribed Topics

* ```/robot_<i>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

* ```/frontier_discovery/frontiers_clusters``` ([multirobotsimulations::Frontiers](../../src/multirobotsimulations/msg/Frontiers.msg))

## Published Topics

* ```<namespace>/mock_communication_model/robots_in_comm``` ([std_msgs::Int8MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Int8MultiArray.html))

<!-- ## Published Transforms

* ```odom``` -->
