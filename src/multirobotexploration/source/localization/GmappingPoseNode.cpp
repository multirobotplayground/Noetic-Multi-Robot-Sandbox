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