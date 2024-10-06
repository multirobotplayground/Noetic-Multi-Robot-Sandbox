/* 
 * Noetic-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
 * Copyright (C) 2023 Alysson Ribeiro da Silva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "MockCommunicationModelNode.h"

/*
 * Node implementation
 */
MockCommunicationModelNode::MockCommunicationModelNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots.");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("comm_dist", aCommDist)) throw std::runtime_error("Could not retrieve comm_dist.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    aNamespace = ros::this_node::getNamespace();

    // initialize communication containers
    aReceivedPoses.assign(aRobots, false);
    aRobotsWorldPoses.assign(aRobots, geometry_msgs::Pose());
    aRelativePoses.assign(aRobots, tf::Vector3());
    aRobotsInComm.data.assign(aRobots, 0);
    LoadRelativePoses(node_handle);

    /*
     * I use lambda in the multi-robot context subscriptions, because it can help 
     * having multiple callbacks, one for each robot, without explicitly
     * defining them, which would required some engineering.
     */
    std::vector<bool>* receivedPosesPtr = &aReceivedPoses;
    std::vector<geometry_msgs::Pose>* robotsWorldPosesPtr = &aRobotsWorldPoses;
    for(int robot = 0; robot < aRobots; ++robot) {    
    aSubscribers.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>("/robot_" + std::to_string(robot) + "/gmapping_pose/world_pose", aQueueSize, 
        [robot, receivedPosesPtr, robotsWorldPosesPtr](multirobotsimulations::CustomPose::ConstPtr msg) {
            robotsWorldPosesPtr->at(robot).position = msg->pose.position;
            robotsWorldPosesPtr->at(robot).orientation = msg->pose.orientation;
            receivedPosesPtr->at(robot) = true;
        }));
    }

    // Advertisers
    aCommunicationModelBroadcaster = node_handle.advertise<std_msgs::Int8MultiArray>(aNamespace + "/mock_communication_model/robots_in_comm", aQueueSize);

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&MockCommunicationModelNode::Update, this)));
}

MockCommunicationModelNode::~MockCommunicationModelNode() {

}

void MockCommunicationModelNode::LoadRelativePoses( ros::NodeHandle& nodeHandle) {
    // read relative start poses parameters
    std::vector<std::map<std::string, double>> poses;
    std::string key = "";
    for (int robot = 0; robot < aRobots; ++robot) {
        key = "/start_pose_robot_" + std::to_string(robot);
        std::map<std::string, double> pose;
        nodeHandle.getParam(key, pose);
        poses.push_back(pose);
        ROS_INFO("[MockCommunicationModelNode]: %s: %f %f %f", key.c_str(), pose["x"], pose["y"], pose["z"]);
    }

    // compute relative poses from file
    tf::Vector3 my_pose(poses[aId]["x"], poses[aId]["y"], poses[aId]["z"]);
    for(int robot = 0; robot < aRobots; ++robot) {
        if(robot!=aId) {
            tf::Vector3 other_pose(poses[robot]["x"], poses[robot]["y"], poses[robot]["z"]);
            tf::Vector3 dir = other_pose - my_pose;
            aRelativePoses[robot] = dir;
            ROS_INFO("[MockCommunicationModelNode] relative to %d: %f %f %f", robot, dir.getX(), dir.getY(), dir.getZ());
        } else {
            aRelativePoses[robot].setX(0.0);
            aRelativePoses[robot].setY(0.0);
            aRelativePoses[robot].setZ(0.0);
            ROS_INFO("[MockCommunicationModelNode] relative to self: %f %f %f", aRelativePoses[robot].getX(), aRelativePoses[robot].getY(), aRelativePoses[robot].getZ());
        }
    }
}

void MockCommunicationModelNode::Update() {
    // translate all the poses here
    tf::Vector3 relative_pose;
    tf::Vector3 my_pose;
    PoseToVector3(aRobotsWorldPoses[aId], my_pose);

    double distance = 0.0;
    for(int robot = 0; robot < aRobots; ++robot) {
        if(aReceivedPoses[robot] == true) {
            PoseToVector3(aRobotsWorldPoses[robot], relative_pose);
            relative_pose += aRelativePoses[robot];

            // check distance to me to see if this the relative positions should be updated
            distance = relative_pose.distance(my_pose);

            if(distance < aCommDist) {
                // set nearby robots
                aRobotsInComm.data[robot] = 1;
            } else {
                aRobotsInComm.data[robot] = 0;
            }
        }
    }

    // send data to the system
    aCommunicationModelBroadcaster.publish(aRobotsInComm);
}

/*
 * Node's main function
 */
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "mockcommunicationmodelnode");
    std::unique_ptr<MockCommunicationModelNode> mockCommunicationModelNode = std::make_unique<MockCommunicationModelNode>();
    ros::spin();
}