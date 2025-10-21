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

#include "GazeboPerfectOdomNode.h"
#include <angles/angles.h>

GazeboPerfectOdomNode::GazeboPerfectOdomNode() 
    : aNodeHandle(), aPrivateNodeHandle("~"), aFirstUpdate(true) {
    
    // Get parameters
    if (!aPrivateNodeHandle.getParam("robot_id", aRobotId)) {
        ROS_WARN("robot_id parameter not set, using default: 0");
        aRobotId = 0;
    }
    
    if (!aPrivateNodeHandle.getParam("publish_rate", aPublishRate)) {
        ROS_WARN("publish_rate parameter not set, using default: 50.0 Hz");
        aPublishRate = 50.0;
    }
    
    // Set frame names
    aRobotName = "pioneer3at_" + std::to_string(aRobotId);
    aWorldFrame = "world";
    aFrameName = "robot_" + std::to_string(aRobotId);
    aOdomFrame = aFrameName + "/odom";
    aBaseFrame = aFrameName + "/base_link";
    
    // Allow custom frame names
    aPrivateNodeHandle.param("world_frame", aWorldFrame, aWorldFrame);
    aPrivateNodeHandle.param("odom_frame", aOdomFrame, aOdomFrame);
    aPrivateNodeHandle.param("base_frame", aBaseFrame, aBaseFrame);
    aPrivateNodeHandle.param("robot_name", aRobotName, aRobotName);
    
    // Initialize ROS components - publish to standard /robot_i/odom topic
    aOdomPublisher = aNodeHandle.advertise<nav_msgs::Odometry>("/robot_" + std::to_string(aRobotId) + "/odom", 1);
    aGazeboClient = aNodeHandle.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    
    // Wait for Gazebo service
    ROS_INFO("[GazeboPerfectOdom] Waiting for Gazebo /get_model_state service...");
    aGazeboClient.waitForExistence();
    ROS_INFO("[GazeboPerfectOdom] Connected to Gazebo service");
    
    // Setup timer
    double update_period = 1.0 / aPublishRate;
    aUpdateTimer = aNodeHandle.createTimer(ros::Duration(update_period), 
                                   &GazeboPerfectOdomNode::updateTimerCallback, this);
    
    ROS_INFO("[GazeboPerfectOdom] Initialized for robot: %s", aRobotName.c_str());
    ROS_INFO("[GazeboPerfectOdom] Publishing to: /robot_%d/odom", aRobotId);
    ROS_INFO("[GazeboPerfectOdom] Frames: %s -> %s -> %s", 
             aWorldFrame.c_str(), aOdomFrame.c_str(), aBaseFrame.c_str());
}

GazeboPerfectOdomNode::~GazeboPerfectOdomNode() {
}

void GazeboPerfectOdomNode::updateTimerCallback(const ros::TimerEvent& event) {
    // Get current time
    ros::Time current_time = ros::Time::now();
    
    // Request model state from Gazebo
    gazebo_msgs::GetModelState srv;
    srv.request.model_name = aRobotName;
    srv.request.relative_entity_name = aWorldFrame;
    
    if (!aGazeboClient.call(srv)) {
        ROS_WARN_THROTTLE(1.0, "[GazeboPerfectOdom] Failed to call /gazebo/get_model_state for %s", 
                         aRobotName.c_str());
        return;
    }
    
    if (!srv.response.success) {
        ROS_WARN_THROTTLE(1.0, "[GazeboPerfectOdom] Gazebo service returned failure for model %s: %s", 
                         aRobotName.c_str(), srv.response.status_message.c_str());
        return;
    }
    
    // Get current pose
    geometry_msgs::Pose current_pose = srv.response.pose;
    
    // Calculate velocity
    geometry_msgs::Twist velocity;
    if (!aFirstUpdate) {
        double delta_time = (current_time - aPrevTime).toSec();
        if (delta_time > 0.0) {
            velocity = calculateVelocity(current_pose, aPrevPose, delta_time);
        }
    } else {
        // First update - zero velocity
        velocity.linear.x = velocity.linear.y = velocity.linear.z = 0.0;
        velocity.angular.x = velocity.angular.y = velocity.angular.z = 0.0;
        aPrevTwist = velocity;  // Initialize previous twist
        aFirstUpdate = false;
    }
    
    // Publish odometry message
    publishOdometry(current_pose, velocity, current_time);
    
    // Publish TF transform
    publishTransform(current_pose, current_time);
    
    // Store current pose and time for next iteration
    aPrevPose = current_pose;
    aPrevTime = current_time;
}

geometry_msgs::Twist GazeboPerfectOdomNode::calculateVelocity(
    const geometry_msgs::Pose& currentPose, 
    const geometry_msgs::Pose& prevPose, 
    double deltaTime) {
    
    geometry_msgs::Twist velocity;
    
    // Ensure minimum time step to avoid division by very small numbers
    if (deltaTime < 1e-6) {
        // Return previous twist for very small time steps
        velocity = aPrevTwist;
        return velocity;
    }
    
    // Linear velocity in world frame
    double dx = currentPose.position.x - prevPose.position.x;
    double dy = currentPose.position.y - prevPose.position.y;
    double dz = currentPose.position.z - prevPose.position.z;
    
    // Convert to robot frame (assuming 2D motion for differential drive robots)
    tf::Quaternion current_quat;
    tf::quaternionMsgToTF(currentPose.orientation, current_quat);
    
    // Get current yaw angle
    double roll, pitch, yaw;
    tf::Matrix3x3(current_quat).getRPY(roll, pitch, yaw);
    
    // Transform linear velocity to robot frame
    velocity.linear.x = dx * cos(yaw) + dy * sin(yaw);  // Forward velocity
    velocity.linear.y = -dx * sin(yaw) + dy * cos(yaw); // Lateral velocity (should be ~0 for diff drive)
    velocity.linear.z = dz / deltaTime;
    
    // Angular velocity calculation with proper angle wrapping
    tf::Quaternion prev_quat;
    tf::quaternionMsgToTF(prevPose.orientation, prev_quat);
    
    // Calculate angular difference more robustly
    tf::Quaternion diff_quat = current_quat * prev_quat.inverse();
    diff_quat.normalize();
    
    // Extract angular velocity components
    double prev_roll, prev_pitch, prev_yaw;
    tf::Matrix3x3(prev_quat).getRPY(prev_roll, prev_pitch, prev_yaw);
    
    // Calculate angular differences with proper wrapping
    double d_roll = angles::shortest_angular_distance(prev_roll, roll);
    double d_pitch = angles::shortest_angular_distance(prev_pitch, pitch);  
    double d_yaw = angles::shortest_angular_distance(prev_yaw, yaw);
    
    velocity.angular.x = d_roll / deltaTime;
    velocity.angular.y = d_pitch / deltaTime;
    velocity.angular.z = d_yaw / deltaTime;
    
    // Apply smoothing filter to reduce noise (simple low-pass filter)
    const double alpha = 0.8; // Smoothing factor
    if (!aFirstUpdate) {
        velocity.linear.x = alpha * velocity.linear.x + (1.0 - alpha) * aPrevTwist.linear.x;
        velocity.linear.y = alpha * velocity.linear.y + (1.0 - alpha) * aPrevTwist.linear.y;
        velocity.angular.z = alpha * velocity.angular.z + (1.0 - alpha) * aPrevTwist.angular.z;
    }
    
    // Store for next iteration
    aPrevTwist = velocity;
    
    return velocity;
}

void GazeboPerfectOdomNode::publishOdometry(const geometry_msgs::Pose& pose, 
                                           const geometry_msgs::Twist& twist, 
                                           const ros::Time& timestamp) {
    nav_msgs::Odometry odom_msg;
    
    // Header
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = aOdomFrame;
    odom_msg.child_frame_id = aBaseFrame;
    
    // Pose - direct from Gazebo (perfect, no drift)
    odom_msg.pose.pose = pose;
    
    // Perfect odometry - zero covariance (infinite confidence)
    for (int i = 0; i < 36; i++) {
        odom_msg.pose.covariance[i] = 0.0;
        odom_msg.twist.covariance[i] = 0.0;
    }
    
    // Twist - calculated from pose differences but smoothed
    odom_msg.twist.twist = twist;
    
    // Publish
    aOdomPublisher.publish(odom_msg);
    
    ROS_DEBUG_THROTTLE(1.0, "[GazeboPerfectOdom] Published perfect odom for %s: pos(%.3f,%.3f,%.3f) vel(%.3f,%.3f)", 
                      aRobotName.c_str(), pose.position.x, pose.position.y, pose.position.z,
                      twist.linear.x, twist.angular.z);
}

void GazeboPerfectOdomNode::publishTransform(const geometry_msgs::Pose& pose, 
                                            const ros::Time& timestamp) {
    geometry_msgs::TransformStamped transform_msg;
    
    // Header
    transform_msg.header.stamp = timestamp;
    transform_msg.header.frame_id = aOdomFrame;
    transform_msg.child_frame_id = aBaseFrame;
    
    // Transform
    transform_msg.transform.translation.x = pose.position.x;
    transform_msg.transform.translation.y = pose.position.y;
    transform_msg.transform.translation.z = pose.position.z;
    transform_msg.transform.rotation = pose.orientation;
    
    // Broadcast
    aTfBroadcaster.sendTransform(transform_msg);
}

void GazeboPerfectOdomNode::run() {
    ROS_INFO("[GazeboPerfectOdom] Node running for robot: %s", aRobotName.c_str());
    ros::spin();
}

// Main function
int main(int argc, char** argv) {
    ros::init(argc, argv, "gazebo_perfect_odom_node");
    
    try {
        GazeboPerfectOdomNode node;
        node.run();
    } catch (const std::exception& e) {
        ROS_ERROR("[GazeboPerfectOdom] Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}