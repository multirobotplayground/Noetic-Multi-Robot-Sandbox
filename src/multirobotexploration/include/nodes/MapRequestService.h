#ifndef MAP_REQUEST_SERVICE_H
#define MAP_REQUEST_SERVICE_H

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

/*
 * ROS and system includes
 */
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>
#include <std_msgs/String.h>

class MapRequestServiceNode {
private:
    // ROS components
    ros::NodeHandle aNodeHandle;
    ros::NodeHandle aPrivateNodeHandle;
    
    // Parameters
    int aRobotId;
    std::string aNamespace;
    int aQueueSize;
    double aRate;
    
    // ROS components
    ros::ServiceServer aMapServiceServer;
    ros::Subscriber aOccupancyGridSubscriber;
    
    // Current occupancy grid
    nav_msgs::OccupancyGrid aCurrentOccupancyGrid;
    bool aHasReceivedMap;
    
    // Callbacks
    void occupancyGridCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg);
    bool mapServiceCallback(nav_msgs::GetMap::Request& request, 
                           nav_msgs::GetMap::Response& response);
    
public:
    MapRequestServiceNode();
    ~MapRequestServiceNode();
    
    void run();
};

#endif // MAP_REQUEST_SERVICE_H