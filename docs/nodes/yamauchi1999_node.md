# Yamauchi1999

This node implements Yamauchi's method from 1999. It relies in a configuration space, frontier discovery services, and a sub goal navigation module. Some of its functionality should be coded as services.

Source: [Yamauchi1999Node.cpp](../../src/multirobotexploration/source/policies/Yamauchi1999Node.cpp)

## Parameters

* ```/robots```

Number of robots in the pack. This is a global parameter.

* ```id```

Id of this robot.

* ```rate```

Main loop rate in hertz.

* ```queue_size```

Queue size of publishers and subscribers.

## Subscribed Topics

* ```<namespace>/c_space``` ([nav_msgs::OccupancyGrid](https://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html))

The configuration space with inflated obstacles for navigation. It must handle other robots, dynamic and static obstacles. The ```<namspace>``` specifies this robot's namespace.

* ```<namespace>/frontier_discovery/frontiers_clusters``` ([multirobotsimulations::Frontiers](../../src/multirobotsimulations/msg/Frontiers.msg))

Frontiers from the frontiers node. They must be filtered and are visibile only for this robot. The ```<namspace>``` specifies this robot's namespace.

* ```<namespace>/gmapping_pose/world_pose``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

Custom pose used throughout the system, it contains the robot id and a pose. The ```<namspace>``` specifies this robot's namespace.

* ```<namespace>/sub_goal_nav/finish``` ([std_msgs::String](../../src/multirobotsimulations/msg/CustomPose.msg))

This topic is used to check wether the sub goal navigation module reached a goal. The ```<namspace>``` specifies this robot's namespace.

* ```<namespace>/explorer/set_idle``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

This topic is used to check wether the sub goal navigation module reached a goal. The ```<namspace>``` specifies this robot's namespace.

* ```<namespace>/explorer/set_exploring``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

This topic is used to check wether the sub goal navigation module reached a goal. The ```<namspace>``` specifies this robot's namespace.

* ```/global_explorer/back_to_base``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

This topic is used to check wether the sub goal navigation module reached a goal. The ```<namspace>``` specifies this robot's namespace.

* ```/global_explorer/set_exploring``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

This topic is used to check wether the sub goal navigation module reached a goal. The ```<namspace>``` specifies this robot's namespace.

## Published Topics

* ```/sub_goal_nav/goal``` ([geometry_msgs::Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))

The goal location to explore to the sub goal navigation module.

* ```/frontier_discovery/compute``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

Communication channel with the frontier discovery module. Used to ask for frontiers.

<!-- ## Published Transforms

* ```odom``` -->
