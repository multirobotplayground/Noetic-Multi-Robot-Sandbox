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

#include "AverageVelocityEstimatorNode.h"

AverageVelocityEstimatorNode::AverageVelocityEstimatorNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("count", aCount)) aCount = 10;
    aNamespace = ros::this_node::getNamespace();

    // subscriptions
    aSubscribers.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>(
        aNamespace + "/world_pose", 
        aQueueSize, 
        std::bind(&AverageVelocityEstimatorNode::WorldPoseCallback, this, std::placeholders::_1)));

    // advertisers
    aAverageVelocityPublisher = node_handle.advertise<std_msgs::Float32>(aNamespace + "/average_velocity", aQueueSize);
    aHasPreviousTime = false;
    aReceivedPosition = false;

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&AverageVelocityEstimatorNode::Update, this)));   
}

AverageVelocityEstimatorNode::~AverageVelocityEstimatorNode() {
    
}

void AverageVelocityEstimatorNode::WorldPoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    aWorldPos.setX(msg->pose.position.x);
    aWorldPos.setY(msg->pose.position.y);

    // initialize last pos, once as soon
    // as a pose is received
    if(!aReceivedPosition) { 
        aLastWorldPos = aWorldPos;
        aReceivedPosition = true;
    }
}

double AverageVelocityEstimatorNode::ComputeAverageVelocity(std::deque<double>& speedArray) {
    double average = 0.0;
    double velocities = static_cast<double>(aVelocityArray.size());
    
    if(velocities > 0) {
        for(std::deque<double>::iterator i = aVelocityArray.begin(); i != aVelocityArray.end(); ++i) average += (*i);
        average /= velocities;
    }

    return average;   
}

void AverageVelocityEstimatorNode::Update() {
    // Get current time
    ros::Time current_time = ros::Time::now();

    // update velocities array
    if(aReceivedPosition) {
        if(aHasPreviousTime) {
            double distance = aWorldPos.distance(aLastWorldPos);
            double time_diff = (current_time - aLastTime).toSec();
            
            if(time_diff > 0.001) {
                // velocity = distance / per unit of time
                double velocity = distance / time_diff;  
                aVelocityArray.push_back(velocity);
                if(aVelocityArray.size() > aCount) aVelocityArray.pop_front();
            }

            aAverageVelocityMsg.data = ComputeAverageVelocity(aVelocityArray);
            aAverageVelocityPublisher.publish(aAverageVelocityMsg);
        } else {
            aHasPreviousTime = true;
        }
        
        aLastWorldPos = aWorldPos;
        aLastTime = current_time;
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "averagespeedestimatornode");
    std::unique_ptr<AverageVelocityEstimatorNode> averagePoseEstimatorNode = std::make_unique<AverageVelocityEstimatorNode>();
    ros::spin();
}