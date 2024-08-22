# Alysson2024Node

This node uses the ```Randomized Social Welfare``` and implements the Intermittent Communication policy from this [research](https://arxiv.org/abs/2309.13494).

Source: [Alysson2024Node.cpp](../../src/multirobotexploration/source/policies/Alysson2024Node.cpp)

## Parameters

* ```/robots```

Number of robots in the pack. This is a global parameter.

* ```id```

Id of this robot.

* ```rate```

Main loop rate in hertz.

* ```queue_size```

Queue size of publishers and subscribers.

* ```/first_rendezvous```

This is a global parameter with the pose, ```{x: 0, y: 0, z: 0}``` , with the first rendezvous location for all sub-teams.

* ```/footprint_robot_<id>```

This is a global parameter with the pose that represents the position of this robot in the rendezvous footprint, where ```<id>``` is the id of this robot.

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

* ```<namespace>/mock_communication_model/robots_in_comm``` ([std_msgs::Int8MultiArray](https://docs.ros.org/en/api/std_msgs/html/msg/Int8MultiArray.html))

This topic receives an integer array from the communication handler, where each position i is 1 if this robot can communicate with i and 0 otherwise. The ```<namspace>``` specifies this robot's namespace.

* ```/robot_<id>/realizing_plan``` ([multirobotsimulations::rendezvous](../../src/multirobotsimulations/msg/rendezvous.msg))

This is a broadcast channel used to receive messages from others that are trying to realize rendezvous plan and that can communicate with this robot. In particullar, this robot creates a subscriber for each robot available during the exploration and identify each subscriber by the ```<id>``` in the topic's name.

* ```/robot_<id>/plan_updater``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

This is a broadcast channel used by any robot to receive rendezvous locations updates for its sub-team.

## Published Topics

* ```/sub_goal_nav/goal``` ([geometry_msgs::Pose](https://docs.ros.org/en/api/geometry_msgs/html/msg/Pose.html))

The goal location to explore to the sub goal navigation module.

* ```/frontier_discovery/compute``` ([std_msgs::String](https://docs.ros.org/en/api/std_msgs/html/msg/String.html))

Communication channel with the frontier discovery module. Used to ask for frontiers.

* ```<namespace>/realizing_plan``` ([multirobotsimulations::rendezvous](../../src/multirobotsimulations/msg/rendezvous.msg))

This topic is used to tell other robots which rendezvous agreement this robot is realizing. It holds this robot's ```id``` and an unique identifier associated with a rendezvous agreement. The ```<namspace>``` specifies this robot's namespace.

* ```<namespace>/plan_updater``` ([multirobotsimulations::CustomPose](../../src/multirobotsimulations/msg/CustomPose.msg))

This robot uses this topic to send a new rendezvous location for the sub-team it is doing the rendezvous, if it is the consensus robot. The consensus robot is the robot if the lowest unique identifier from the sub-team that is doing the rendezvous. Rendezvous also have unique identifiers.


<!-- ## Published Transforms

* ```odom``` -->
