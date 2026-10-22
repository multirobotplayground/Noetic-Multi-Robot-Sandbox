/*
 * Noetic-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
 * Copyright (C) 2025 Alysson Ribeiro da Silva
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

#include "MapRequestService.h"

MapRequestServiceNode::MapRequestServiceNode() 
    : aNodeHandle(), aPrivateNodeHandle("~"), aHasReceivedMap(false) {
    
    // Get parameters
    if (!aPrivateNodeHandle.getParam("robot_id", aRobotId)) {
        ROS_WARN("[MapRequestService] robot_id parameter not set, using default: 0");
        aRobotId = 0;
    }
    if (!aPrivateNodeHandle.getParam("rate", aRate)) aRate = 10.0;
    if (!aPrivateNodeHandle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    
    // Get namespace
    aNamespace = ros::this_node::getNamespace();
    
    // Subscribe to the robot's occupancy grid (c_space or fusion)
    aOccupancyGridSubscriber = aNodeHandle.subscribe<nav_msgs::OccupancyGrid>(
        aNamespace + "/map", 
        aQueueSize, 
        &MapRequestServiceNode::occupancyGridCallback, 
        this);
    
    // Create the map request service
    std::string serviceName = aNamespace + "/service_request_map";
    aMapServiceServer = aNodeHandle.advertiseService(
        serviceName, 
        &MapRequestServiceNode::mapServiceCallback, 
        this);
    
    ROS_INFO("[MapRequestService] Node initialized for robot %d", aRobotId);
    ROS_INFO("[MapRequestService] Service available at: %s", serviceName.c_str());
    ROS_INFO("[MapRequestService] Subscribing to: %s", (aNamespace + "/map").c_str());
}

MapRequestServiceNode::~MapRequestServiceNode() {
}

void MapRequestServiceNode::occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg) {
    // Store the current occupancy grid
    aCurrentOccupancyGrid = *msg;
    
    if (!aHasReceivedMap) {
        aHasReceivedMap = true;
        ROS_INFO("[MapRequestService] Received first occupancy grid for robot %d", aRobotId);
    }
    
    ROS_DEBUG_THROTTLE(5.0, "[MapRequestService] Updated occupancy grid for robot %d (size: %dx%d)", 
                      aRobotId, aCurrentOccupancyGrid.info.width, aCurrentOccupancyGrid.info.height);
}

bool MapRequestServiceNode::mapServiceCallback(nav_msgs::GetMap::Request& request, 
                                              nav_msgs::GetMap::Response& response) {
    
    ROS_INFO("[MapRequestService] Map request received for robot %d", aRobotId);
    
    if (!aHasReceivedMap || aCurrentOccupancyGrid.data.empty()) {
        ROS_WARN("[MapRequestService] No occupancy grid available yet for robot %d", aRobotId);
        return false;
    }
    
    // Return the current occupancy grid
    response.map = aCurrentOccupancyGrid;
    
    ROS_INFO("[MapRequestService] Returning map for robot %d (size: %dx%d, resolution: %.3f)", 
             aRobotId, response.map.info.width, response.map.info.height, response.map.info.resolution);
    
    return true;
}

void MapRequestServiceNode::run() {
    ROS_INFO("[MapRequestService] Node running for robot %d", aRobotId);
    ros::spin();
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "map_request_service_node");
    
    try {
        MapRequestServiceNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("[MapRequestService] Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}