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

#ifndef GAZEBO_PERFECT_ODOM_NODE_H
#define GAZEBO_PERFECT_ODOM_NODE_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <angles/angles.h>

class GazeboPerfectOdomNode {
public:
    GazeboPerfectOdomNode();
    ~GazeboPerfectOdomNode();
    
    void run();

private:
    ros::NodeHandle aNodeHandle;
    ros::NodeHandle aPrivateNodeHandle;
    
    // Parameters
    int aRobotId;
    std::string aRobotName;
    std::string aWorldFrame;
    std::string aFrameName;
    std::string aOdomFrame;
    std::string aBaseFrame;
    double aPublishRate;
    
    // ROS components
    ros::Publisher aOdomPublisher;
    ros::ServiceClient aGazeboClient;
    ros::Timer aUpdateTimer;
    tf::TransformBroadcaster aTfBroadcaster;
    
    // Previous pose for velocity calculation
    geometry_msgs::Pose aPrevPose;
    geometry_msgs::Twist aPrevTwist;
    ros::Time aPrevTime;
    bool aFirstUpdate;
    
    // Methods
    void updateTimerCallback(const ros::TimerEvent& event);
    geometry_msgs::Twist calculateVelocity(const geometry_msgs::Pose& currentPose, 
                                          const geometry_msgs::Pose& prevPose, 
                                          double deltaTime);
    void publishOdometry(const geometry_msgs::Pose& pose, 
                        const geometry_msgs::Twist& twist, 
                        const ros::Time& timestamp);
    void publishTransform(const geometry_msgs::Pose& pose, const ros::Time& timestamp);
};

#endif // GAZEBO_PERFECT_ODOM_NODE_H