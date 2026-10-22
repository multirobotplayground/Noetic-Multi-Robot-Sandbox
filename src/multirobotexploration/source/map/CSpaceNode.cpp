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

#include "CSpaceNode.h"

CSpaceNode::CSpaceNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("free_inflation_radius", aFreeInflateRadius)) aFreeInflateRadius = 0.7;
    if(!node_handle.getParam("ocu_inflation_radius", aOccuInflateRadius)) aOccuInflateRadius = 0.5;
    if(!node_handle.getParam("mark_robots_for_planning", aMarkRobotsForLocalPlanning)) aMarkRobotsForLocalPlanning = false;
    if(!node_handle.getParam("robots", aRobots)) aRobots = 3;
    aNamespace = ros::this_node::getNamespace();

    aHasOcc = false;
    aHasComm = false;
    aReceivedPoses.assign(aRobots, false);
    aWorldPoses.assign(aRobots, geometry_msgs::Pose());

    for(int robot = 0; robot <aRobots;++robot) {
        std::vector<geometry_msgs::Pose>* robotsWorldPosesPtr = &aWorldPoses;
        std::vector<bool>* receivedPosesPtr = &aReceivedPoses;
        aSubscribers.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>(
                                "/robot_" + std::to_string(robot) + "/world_pose", 
                                aQueueSize,
                                [this, robot, robotsWorldPosesPtr, receivedPosesPtr](multirobotsimulations::CustomPose::ConstPtr msg) {
                                    robotsWorldPosesPtr->at(robot).position = msg->pose.position;
                                    robotsWorldPosesPtr->at(robot).orientation = msg->pose.orientation;
                                    receivedPosesPtr->at(robot) = true;
                                })
                            );
    }
    aSubscribers.push_back(
                node_handle.subscribe<std_msgs::Int8MultiArray>(
                    aNamespace + "/mock_communication_model/robots_in_comm", 
                    aQueueSize,
                    std::bind(&CSpaceNode::CommunicationsCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(node_handle.subscribe<nav_msgs::OccupancyGrid>(
                            aNamespace + "/map", 
                            aQueueSize, 
                            std::bind(&CSpaceNode::OccCallback, this, std::placeholders::_1)));

    // Advertisers
    aCspacePublisher = node_handle.advertise<nav_msgs::OccupancyGrid>(aNamespace + "/c_space", aQueueSize);

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&CSpaceNode::Update, this)));
}

CSpaceNode::~CSpaceNode() {

}


void CSpaceNode::CommunicationsCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    // Update current communication state
    if(!aHasComm) aHasComm = true;
    aRobotsInCommMsg.data.assign(msg->data.begin(), msg->data.end());
}

void CSpaceNode::OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    if(!aHasOcc) aHasOcc = true;
    aOccMsg.data.assign(msg->data.begin(), msg->data.end());
    aOccMsg.info = msg->info;
    aOccMsg.header = msg->header;
}

void CSpaceNode::Inflate(nav_msgs::OccupancyGrid& occ,
            const double& freeInflationRadius,
            const double& occupiedInflationRadius, 
            const int8_t& occupancyThreshold,
            const int8_t& freeThreshold,
            const int8_t& occupiedValue,
            const int8_t& freeVal) {
    aCspaceMsg.data.assign(occ.data.begin(), occ.data.end());
    aCspaceMsg.header = occ.header;
    aCspaceMsg.info = occ.info;

    int index;
    int8_t val;
    int8_t raw_val;
    int width = occ.info.width;
    int height = occ.info.height;
    int irp_occu = static_cast<int>(occupiedInflationRadius / occ.info.resolution);
    int irp_free = static_cast<int>(freeInflationRadius / occ.info.resolution);
    for(int y = 0; y < occ.info.height; ++y) {
        for(int x = 0; x < occ.info.width; ++x) {
            index = y * width + x;
            val = occ.data[index];
            if(val >= 0 && val < freeThreshold)
                ApplyMask(x, y, irp_free, aCspaceMsg.data, freeVal, width, height);
        }
    }

    for(int y = 0; y < occ.info.height; ++y) {
        for(int x = 0; x < occ.info.width; ++x) {
            index = y * width + x;
            val = occ.data[index];
            if(val > occupancyThreshold)
                ApplyMask(x, y, irp_occu, aCspaceMsg.data, occupiedValue, width, height);
        }
    }
}

void CSpaceNode::Update() {
    if(!aHasOcc || !aReceivedPoses[aId] || aHasComm == false) return;

    /*
     * Clear dynamic trajectories in local map
     */
    Inflate(aOccMsg, aFreeInflateRadius, aOccuInflateRadius);
    Vec2i occ_pos;
    tf::Vector3 world_pose(aWorldPoses[aId].position.x, 
                                    aWorldPoses[aId].position.y, 
                                    aWorldPoses[aId].position.z);

    WorldToMap(aCspaceMsg, world_pose, occ_pos);
    int irp_free = static_cast<int>(aFreeInflateRadius / aOccMsg.info.resolution);
    ApplyMask(occ_pos.x, occ_pos.y, irp_free, aCspaceMsg.data, 0, aCspaceMsg.info.width, aCspaceMsg.info.height);

    /*
    * Inflate for planner here
    *
    */
   if(aMarkRobotsForLocalPlanning) {
        for(int i = 0; i < aRobots; ++i) {
            if(i == aId) continue;
            if(!aReceivedPoses[i]) continue;
            if(aRobotsInCommMsg.data[i] == 0) continue;
            tf::Vector3 other_world_pose(aWorldPoses[i].position.x, 
                                            aWorldPoses[i].position.y, 
                                            aWorldPoses[i].position.z);
            //ROS_INFO("Robot %d is in communication with robot %d", aId, i);
            //ROS_INFO("Robot %d world pose: [%f, %f, %f]", i, other_world_pose.getX(), other_world_pose.getY(), other_world_pose.getZ());
            Vec2i other_occ_pos;
            WorldToMap(aCspaceMsg, other_world_pose, other_occ_pos);
            aCspaceMsg.data[other_occ_pos.y * aCspaceMsg.info.width + other_occ_pos.x] = 100; // mark robot's position
        }
    }

    // publish the occ
    aCspacePublisher.publish(aCspaceMsg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "cspacenode");
    std::unique_ptr<CSpaceNode> relativePoseEstimatorNode = std::make_unique<CSpaceNode>();
    ros::spin();
}