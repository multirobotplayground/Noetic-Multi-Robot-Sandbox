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

#include "LaserToWorldNode.h"

LaserToWorldNode::LaserToWorldNode() {
    ros::NodeHandle node_handle("~");
    aHasLidar = false;
    aHasOccInfo = false;
    aHasPose = false;

    // load all parameters
    double x,y,z,roll,pitch,yaw;
    if(!node_handle.getParam("x", x)) x = 0.0;
    if(!node_handle.getParam("y", y)) y = 0.0;
    if(!node_handle.getParam("z", z)) z = 0.0;
    if(!node_handle.getParam("roll", roll)) roll = 0.0;
    if(!node_handle.getParam("pitch", pitch)) pitch = 0.0;
    if(!node_handle.getParam("yaw", yaw)) yaw = 0.0;
    if(!node_handle.getParam("rate", aRate)) aRate = 5;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    aNamespace = ros::this_node::getNamespace();

    aLidarPosition = tf::Vector3(x,y,0.0);
    aLidarOrientation = tf::createQuaternionFromRPY(roll,pitch,yaw);

    // subscriptions
    aSubscribers.push_back(
        node_handle.subscribe<sensor_msgs::LaserScan>(
            aNamespace + "/scan", 
            aQueueSize, 
            std::bind(&LaserToWorldNode::LaserCapture, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::CustomPose>(
            aNamespace + "/gmapping_pose/world_pose", 
            aQueueSize, 
            std::bind(&LaserToWorldNode::EstimatePoseWorldCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<nav_msgs::OccupancyGrid>(
            aNamespace + "/map", 
            aQueueSize, 
            std::bind(&LaserToWorldNode::OccupancyGridCallback, this, std::placeholders::_1)));

    // advertisers
    aLidarPublisher = node_handle.advertise<geometry_msgs::PoseArray>(aNamespace + "/laser_to_world/laser_world", aQueueSize);
    aOccLidarPublisher = node_handle.advertise<geometry_msgs::PoseArray>(aNamespace + "/laser_to_world/laser_occ", aQueueSize);

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&LaserToWorldNode::Update, this)));
}

LaserToWorldNode::~LaserToWorldNode() {
    
}

void LaserToWorldNode::EstimatePoseWorldCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    aRobotWorldPosition.setX(msg->pose.position.x);
    aRobotWorldPosition.setY(msg->pose.position.y);
    aRobotWorldPosition.setZ(0.0);
    aRobotYaw = tf::getYaw(msg->pose.orientation);
    aHasPose = true;
}

void LaserToWorldNode::OccupancyGridCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    aOccInfo.info = msg->info;
    aHasOccInfo = true;
}

void LaserToWorldNode::LaserCapture(sensor_msgs::LaserScan::ConstPtr msg) {
    if(!aHasPose || !aHasOccInfo) return

    // clear previous readings
    aWorldReadings.clear();
    aOccReadings.clear();
    double increment   = msg->angle_increment;
    double theta       = msg->angle_min;
    tf::Vector3 rot_axis(0,0,1);

    // compute laser robot frame exact positions
    for(size_t beam = 0; beam < msg->ranges.size(); ++beam) {
        double range = msg->ranges[beam];

        // crop lasers with threshold
        if(range > msg->range_max - aLidarError) {
            theta += increment;
            continue;
        }

        // do rotation on YZ plane normal X
        tf::Vector3 laser_vec(1,0,0);

        // rotate arround Z
        laser_vec = laser_vec.rotate(rot_axis, theta);

        // extend vector with range in meters
        tf::Vector3 laser_world = laser_vec * range + tf::Vector3(0.3,0.0,0.0);
        laser_world = laser_world.rotate(tf::Vector3(0,0,1), aRobotYaw);
        laser_world = laser_world.rotate(tf::Vector3(0,0,1), tf::getYaw(aLidarOrientation));
        laser_world += aRobotWorldPosition;

        geometry_msgs::Pose laser_pose_world;
        laser_pose_world.position.x = laser_world.getX();
        laser_pose_world.position.y = laser_world.getY();
        laser_pose_world.position.z = range;

        // convert world laser coordinates into occ laser coordinates
        tf::Vector3 world_occ;
        WorldToMap(aOccInfo, laser_world, world_occ);
        geometry_msgs::Pose laser_pose_occ;
        laser_pose_occ.position.x = world_occ.getX();
        laser_pose_occ.position.y = world_occ.getY();
        laser_pose_occ.position.z = range;

        // add laser point to readings array
        aWorldReadings.push_back(laser_pose_world);
        aOccReadings.push_back(laser_pose_occ);

        // increment ray angle
        theta += increment;
    }

    aHasLidar = true;
}

void LaserToWorldNode::Update() {
    if(!aHasLidar) return;

    aWorldLidarMsg.poses.assign(aWorldReadings.begin(), aWorldReadings.end());
    aOccLidarMsg.poses.assign(aOccReadings.begin(), aOccReadings.end());

    aLidarPublisher.publish(aWorldLidarMsg);
    aOccLidarPublisher.publish(aOccLidarMsg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "lasertoworldnode");
    std::unique_ptr<LaserToWorldNode> laserToWorldNode = std::make_unique<LaserToWorldNode>();
    ros::spin();
}