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

#include "GmappingPoseNode.h"

GmappingPoseNode::GmappingPoseNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    aNamespace = ros::this_node::getNamespace();

    // transform listener
    aTFBuffer = std::make_shared<tf2_ros::Buffer>();
    aTFListener = std::make_shared<tf2_ros::TransformListener>(*aTFBuffer);
    
    // transform frame names
    std::string ns_clean = aNamespace;
    ns_clean.erase(remove(ns_clean.begin(), ns_clean.end(), '/'), ns_clean.end());
    ROS_INFO("[gmapping_pose] lookup transform name: %s from: %s", ns_clean.c_str(), aNamespace.c_str());

    if(ns_clean.size() == 0) {
        aTFMap = "map";
        aTFBaseLink = "base_link";
    } else {
        aTFMap = ns_clean + "/map";
        aTFBaseLink = ns_clean +  "/base_link"; 
    }
    ROS_INFO("[gmapping_pose] looking for tf: %s", aTFMap.c_str());

    // subscriptions
    aSubscribers.push_back(
        node_handle.subscribe<nav_msgs::OccupancyGrid>(
            aNamespace + "/map", 
            aQueueSize, 
            std::bind(&GmappingPoseNode::OccCallback, this, std::placeholders::_1)));

    // advertisers
    aPosePublisher = node_handle.advertise<multirobotsimulations::CustomPose>(aNamespace + "/gmapping_pose/world_pose", aQueueSize);
    aPoseStampedPublisher = node_handle.advertise<geometry_msgs::PoseStamped>(aNamespace + "/gmapping_pose/pose_stamped", aQueueSize);
    aPathPublisher = node_handle.advertise<nav_msgs::Path>(aNamespace + "/path", aQueueSize);

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&GmappingPoseNode::Update, this)));
}

GmappingPoseNode::~GmappingPoseNode() {

}

void GmappingPoseNode::OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    aOcc.info = msg->info;
}

void GmappingPoseNode::Update() {
    /*
     *  get current estimated pose from gmapping
     */
    geometry_msgs::TransformStamped map_base_link;
    try{
        /*
         * Transform lookup do the transformation between the map reference frame 
         * and the base_link reference frame
         */
        map_base_link = aTFBuffer->lookupTransform(aTFMap, aTFBaseLink, ros::Time(0));

        /*
         * Process captured transform
         */
        aPoseStamped.header = map_base_link.header;
        aPoseStamped.pose.position.x = map_base_link.transform.translation.x;
        aPoseStamped.pose.position.y = map_base_link.transform.translation.y;
        aPoseStamped.pose.position.z = map_base_link.transform.translation.z;

        /*
         * Prepare pose to be published with captured transformation data
         */
        aPose.pose.position.x = map_base_link.transform.translation.x;
        aPose.pose.position.y = map_base_link.transform.translation.y;
        aPose.pose.position.z = map_base_link.transform.translation.z;
        aPose.pose.orientation = map_base_link.transform.rotation;
        aPose.robot_id = aId;

        /*
         * Assemble path
         */
        aPath.header = map_base_link.header;
        aPath.poses.push_back(aPoseStamped);

        /*
         * Broadcast all data
         */
        aPoseStampedPublisher.publish(aPoseStamped);
        aPosePublisher.publish(aPose);
        aPathPublisher.publish(aPath);
    } catch(tf2::TransformException &ex) {
        ROS_INFO("[gmapping_pose] %s", ex.what());
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "gmappingposenode");
    std::unique_ptr<GmappingPoseNode> gmappingPoseNode = std::make_unique<GmappingPoseNode>();
    ros::spin();
}