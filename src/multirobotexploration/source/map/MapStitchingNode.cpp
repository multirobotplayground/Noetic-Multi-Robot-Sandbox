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


#include "MapStitchingNode.h"

MapStitchingNode::MapStitchingNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots.");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    aNamespace = ros::this_node::getNamespace();

    aReceivedRelativePoses = false;
    aDirty = true;

    // containers initialization
    aRobotsInCommMsg.data.assign(aRobots, 0);
    aRobotsOcc.assign(aRobots, nav_msgs::OccupancyGrid());  
    aReceivedOccs.assign(aRobots, false);

    // subscriptions
    for(int robot = 0; robot < aRobots; ++robot) {
        if(robot == aId) continue;

        nav_msgs::OccupancyGrid* occPtr = &aRobotsOcc[robot];
        std_msgs::Int8MultiArray* commPtr = &aRobotsInCommMsg;
        std::vector<bool>* receivedFlagPtr = &aReceivedOccs;
        aSubscribers.push_back(
            node_handle.subscribe<nav_msgs::OccupancyGrid>(
                    "/robot_" + std::to_string(robot) + "/fusion", 
                    aQueueSize, 
                    [occPtr, receivedFlagPtr, commPtr, robot](nav_msgs::OccupancyGrid::ConstPtr msg){
                        // mock communication to merge maps
                        // if this flag is 0, then the 'robot' cannot
                        // share its map
                        if(commPtr->data[robot] == 0) return;

                        // otherwise, update the last received map
                        occPtr->data.assign(msg->data.begin(), msg->data.end());
                        occPtr->info = msg->info;
                        occPtr->header = msg->header;

                        // set received flag to true
                        receivedFlagPtr->at(robot) = true;
                    }));
    }

    aSubscribers.push_back(
                node_handle.subscribe<std_msgs::Int8MultiArray>(
                    aNamespace + "/mock_communication_model/robots_in_comm", 
                    aQueueSize,
                    std::bind(&MapStitchingNode::CommunicationsCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
                node_handle.subscribe<geometry_msgs::PoseArray>(
                    aNamespace + "/relative_pose_estimator/relative_start", 
                    aQueueSize,
                    std::bind(&MapStitchingNode::RelativeStartingPosesCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
                node_handle.subscribe<nav_msgs::OccupancyGrid>(
                    aNamespace + "/map", 
                    aQueueSize,
                    std::bind(&MapStitchingNode::OccCallback, this, std::placeholders::_1)));

    // advertisers
    aFusionPublisher = node_handle.advertise<nav_msgs::OccupancyGrid>(aNamespace + "/fusion", aQueueSize);

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&MapStitchingNode::Update, this)));
}

MapStitchingNode::~MapStitchingNode() {

}

void MapStitchingNode::CommunicationsCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    aRobotsInCommMsg.data.assign(msg->data.begin(), msg->data.end());
}

void MapStitchingNode::RelativeStartingPosesCallback(geometry_msgs::PoseArray::ConstPtr msg) {
    aRobotsRelativePosesMsg.header = msg->header;
    aRobotsRelativePosesMsg.poses.assign(msg->poses.begin(), msg->poses.end());
    aReceivedRelativePoses = true;
}

void MapStitchingNode::OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    aRobotsOcc[aId].data.assign(msg->data.begin(), msg->data.end());
    aRobotsOcc[aId].info = msg->info;
    aRobotsOcc[aId].header = msg->header;
    aReceivedOccs[aId] = true;
}

void MapStitchingNode::Fusemaps(nav_msgs::OccupancyGrid& occ, 
                                    nav_msgs::OccupancyGrid& other, 
                                    geometry_msgs::Pose& relative, 
                                    const bool& replace) {
    // transformation does not invert indexes
    int x, y;
    int8_t val;
    int8_t self_val;
    tf::Vector3 world_relative;
    tf::Vector3 world_origin;
    tf::Vector3 map_origin;
    tf::Vector3 map_relative;
    tf::Vector3 relative_transform;

    PoseToVector3(relative, world_relative);
    WorldToMap(occ, world_origin, map_origin);
    WorldToMap(occ, world_relative, map_relative);
    relative_transform = map_relative - map_origin;

    for(size_t index = 0; index < occ.data.size(); ++index) {
        x = index % occ.info.width;
        y = index / occ.info.width;
        
        int res_x = x + relative_transform.getX();
        int res_y = y + relative_transform.getY();

        int other_index = res_y * occ.info.width + res_x;

        self_val = occ.data[index];
        val = other.data[index];
        if(res_x > 0 && res_x < occ.info.width && res_y > 0 && res_y < occ.info.height) {
           if(replace == true) {
                if(val != -1) {
                    occ.data[other_index] = val;
                }
            } else {
                if(self_val == -1) {
                    if(val != -1)  {
                        occ.data[other_index] = val;
                    }
                }
            }
        }
    }
}

void MapStitchingNode::Update() {
        if(aReceivedOccs[aId] == false || aReceivedRelativePoses == false) return;

        // initialize first occupancy grid
        if(aDirty) {
            aFusionMsg.info = aRobotsOcc[aId].info;
            aFusionMsg.header = aRobotsOcc[aId].header;
            aFusionMsg.data.assign(aRobotsOcc[aId].data.size(), -1);
            aDirty = false;
        }

        // do the map transformation here
        // other adjustments can be made
        // but for the purpose of several experiments,
        // initial translation should be enough
        for(int robot = 0; robot < aRobots; ++robot) {
            // do not process my map
            if(robot == aId || aReceivedOccs[robot] == false) continue;

            // just fuse the maps
            Fusemaps(aFusionMsg, aRobotsOcc[robot], aRobotsRelativePosesMsg.poses[robot], false);
        }

        // copy my map on top of everyone
        Fusemaps(aFusionMsg, aRobotsOcc[aId], aRobotsRelativePosesMsg.poses[aId], true);

        // publish fusion, this robot's map will always be on top
        aFusionPublisher.publish(aFusionMsg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "mapstitchingnode");
    std::unique_ptr<MapStitchingNode> mapStitchingNode = std::make_unique<MapStitchingNode>();
    ros::spin();
}
