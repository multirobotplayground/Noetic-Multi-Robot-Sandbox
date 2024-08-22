/*
 * Copyright (c) 2020, Alysson Ribeiro da Silva
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. All advertising materials mentioning features or use of this software must
 *    display the following acknowledgement:
 *    This product includes software developed by Alysson Ribeiro da Silva.
 * 
 * 4. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * 5. The source and the binary form, and any modifications made to them
 *    may not be used for the purpose of training or improving machine learning
 *    algorithms, including but not limited to artificial intelligence, natural
 *    language processing, or data mining. This condition applies to any derivatives,
 *    modifications, or updates based on the Software code. Any usage of the source
 *    or the binary form in an AI-training dataset is considered a breach of
 *    this License.
 * 
 * THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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