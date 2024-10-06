/* 
 * Noetic-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
 * Copyright (C) 2020 Alysson Ribeiro da Silva
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

#include "RelativePoseEstimatorNode.h"

RelativePoseEstimatorNode::RelativePoseEstimatorNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve robots.");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Coult not retrieve id.");
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    aNamespace = ros::this_node::getNamespace();

    // variables initialization
    aSeq = 0;

    // initialize all containers to hold robots' information
    aRobotsWorldPoses.assign(aRobots, geometry_msgs::Pose());
    aRelativePoses.assign(aRobots, tf::Vector3());
    aReceivedPoses.assign(aRobots, false);
    aRobotsRelativePosesMsg.poses.assign(aRobots, geometry_msgs::Pose());
    aRobotsRelStartingPosMsg.poses.assign(aRobots, geometry_msgs::Pose());
    aRobotsRelativeDistancesMsg.data.assign(aRobots, 0.0);
    aRobotsInCommMsg.data.assign(aRobots, 0);

    // load relative poses container
    LoadRelativePoses(node_handle);

    // subscriptions
    std::vector<geometry_msgs::Pose>* robotsWorldPosesPtr = &aRobotsWorldPoses;
    std::vector<bool>* receivedPosesPtr = &aReceivedPoses;
    for(int robot = 0; robot < aRobots; ++robot) {
        aSubscribers.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>("/robot_" + std::to_string(robot) +"/gmapping_pose/world_pose", aQueueSize, 
            [receivedPosesPtr, robotsWorldPosesPtr, robot](multirobotsimulations::CustomPose::ConstPtr msg) {
                robotsWorldPosesPtr->at(robot).position = msg->pose.position;
                robotsWorldPosesPtr->at(robot).orientation = msg->pose.orientation;
                receivedPosesPtr->at(robot) = true;
            }));
    }

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Int8MultiArray>(
            aNamespace + "/mock_communication_model/robots_in_comm", 
            aQueueSize,
            std::bind(&RelativePoseEstimatorNode::CommCallback, this, std::placeholders::_1)));

    // advertisers
    aStartRelativePosesPublisher = node_handle.advertise<geometry_msgs::PoseArray>(aNamespace + "/relative_pose_estimator/relative_start", aQueueSize);
    aRelativePosesPublisher = node_handle.advertise<geometry_msgs::PoseArray>(aNamespace + "/relative_pose_estimator/relative_poses", aQueueSize);
    aDistancesPublisher = node_handle.advertise<std_msgs::Float64MultiArray>(aNamespace + "/relative_pose_estimator/distances", aQueueSize);
    aNearMarkerPublisher = node_handle.advertise<visualization_msgs::Marker>(aNamespace + "/relative_pose_estimator/pose_markers", aQueueSize);
    aFarMarkerPublisher = node_handle.advertise<visualization_msgs::Marker>(aNamespace + "/relative_pose_estimator/pose_far_markers", aQueueSize);

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&RelativePoseEstimatorNode::Update, this)));
}

RelativePoseEstimatorNode::~RelativePoseEstimatorNode() {
}

void RelativePoseEstimatorNode::PrepareMarkers(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq) {
    input.id = id;
    input.header.frame_id = std::string("robot_") + std::to_string(id) + std::string("/map");
    input.header.stamp = ros::Time().now();
    input.header.seq = seq;
    input.ns = ns;
    input.points.clear();
    input.type = visualization_msgs::Marker::CUBE_LIST;
    input.action = visualization_msgs::Marker::MODIFY;
    input.pose.orientation.x = 0.0;
    input.pose.orientation.y = 0.0;
    input.pose.orientation.z = 0.0;
    input.pose.orientation.w = 1.0;
    input.scale.x = 0.5;
    input.scale.y = 0.5;
    input.scale.z = 0.5;
    input.color.a = 1.0;
    input.color.r = 1.0;
    input.color.g = 0.3;
    input.color.b = 0.1;
    input.lifetime = ros::Duration(60);
}

void RelativePoseEstimatorNode::SetNear(visualization_msgs::Marker& input) {
    input.color.a = 1.0;
    input.color.r = 0.0;
    input.color.g = 1.0;
    input.color.b = 0.1;
}

void RelativePoseEstimatorNode::SetFar(visualization_msgs::Marker& input) {
    input.color.a = 1.0;
    input.color.r = 1.0;
    input.color.g = 0.0;
    input.color.b = 0.1;
}

void RelativePoseEstimatorNode::CommCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    aRobotsInCommMsg.data.assign(msg->data.begin(), msg->data.end());
}

void RelativePoseEstimatorNode::LoadRelativePoses(ros::NodeHandle& nodeHandle) {
    // read relative start poses parameters
    std::vector<std::map<std::string, double>> poses;
    std::string key = "";
    for (int robot = 0; robot < aRobots; ++robot) {
        key = "/start_pose_robot_" + std::to_string(robot);
        std::map<std::string, double> pose;
        nodeHandle.getParam(key, pose);
        poses.push_back(pose);
        ROS_INFO("[Relative_pose_estimator]: %s: %f %f %f", key.c_str(), pose["x"], pose["y"], pose["z"]);
    }

    // compute relative poses from file
    tf::Vector3 my_pose(poses[aId]["x"], poses[aId]["y"], poses[aId]["z"]);
    for(int robot = 0; robot < aRobots; ++robot) {
        if(robot!=aId) {
            tf::Vector3 other_pose(poses[robot]["x"], poses[robot]["y"], poses[robot]["z"]);
            tf::Vector3 dir = other_pose - my_pose;
            aRelativePoses[robot] = dir;
            Vector3ToPose(dir, aRobotsRelStartingPosMsg.poses[robot]);
            ROS_INFO("[Relative_pose_estimator] relative to %d: %f %f %f", robot, dir.getX(), dir.getY(), dir.getZ());
        } else {
            aRelativePoses[robot].setX(0.0);
            aRelativePoses[robot].setY(0.0);
            aRelativePoses[robot].setZ(0.0);
            ROS_INFO("[Relative_pose_estimator] relative to self: %f %f %f", aRelativePoses[robot].getX(), aRelativePoses[robot].getY(), aRelativePoses[robot].getZ());
        }
    }
}

void RelativePoseEstimatorNode::Update() {
    // create a new cluster marker msg to be sent
    PrepareMarkers(aClusterMarkerMsg, aNamespace.c_str(), aId, aSeq);
    SetNear(aClusterMarkerMsg);

    PrepareMarkers(aClusterMarkerFarMsg, aNamespace.c_str(), aId, aSeq);
    SetFar(aClusterMarkerFarMsg);

    // translate all the poses here
    tf::Vector3 relative_pose;
    tf::Vector3 mine;
    PoseToVector3(aRobotsWorldPoses[aId], mine);
    double distance = 0.0;
    for(int robot = 0; robot < aRobots; ++robot) {
        if(robot == aId) continue;
        
        if(aReceivedPoses[robot] == true) {
            PoseToVector3(aRobotsWorldPoses[robot], relative_pose); 
            relative_pose += aRelativePoses[robot];

            geometry_msgs::Point p;

            if(aRobotsInCommMsg.data[robot] == 1) {
                // store into pose array to be sent
                Vector3ToPose(relative_pose, aRobotsRelativePosesMsg.poses[robot]);
                p.z = 0.25;
                p.x = aRobotsRelativePosesMsg.poses[robot].position.x;
                p.y = aRobotsRelativePosesMsg.poses[robot].position.y;

                // update marker array for relative poses
                // and do it only for the other robots
                aClusterMarkerMsg.points.push_back(p);
                
                // check distance to me to see if this the relative positions should be updated
                distance = relative_pose.distance(mine);
            } else {
                p.z = 0.25;
                p.x = aRobotsRelativePosesMsg.poses[robot].position.x;
                p.y = aRobotsRelativePosesMsg.poses[robot].position.y;

                aClusterMarkerFarMsg.points.push_back(p);
                distance = 10000000.0;
            }

            // also set the distances array
            aRobotsRelativeDistancesMsg.data[robot] = distance;
        }
    }

    // send data to the system
    aStartRelativePosesPublisher.publish(aRobotsRelStartingPosMsg);
    aRelativePosesPublisher.publish(aRobotsRelativePosesMsg);
    aDistancesPublisher.publish(aRobotsRelativeDistancesMsg);
    aNearMarkerPublisher.publish(aClusterMarkerMsg);
    aFarMarkerPublisher.publish(aClusterMarkerFarMsg);

    aSeq += 1;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "relativeposeestimatornode");
    std::unique_ptr<RelativePoseEstimatorNode> relativePoseEstimatorNode = std::make_unique<RelativePoseEstimatorNode>();
    ros::spin();
}