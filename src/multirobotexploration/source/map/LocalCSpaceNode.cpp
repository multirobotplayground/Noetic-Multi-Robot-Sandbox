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

#include "LocalCSpaceNode.h"

LocalCSpaceNode::LocalCSpaceNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("lidar_sources", aLidarSources)) throw std::runtime_error("Could not retrieve lidar_sources.");
    if(!node_handle.getParam("local_view_size", aLocalViewSize)) aLocalViewSize = 5.0;
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("max_lidar_range", aLidarRange)) aLidarRange = 10.0;
    if(!node_handle.getParam("free_inflation_radius", aFreeInflateRadius)) aFreeInflateRadius = 0.7;
    if(!node_handle.getParam("ocu_inflation_radius", aOccuInflateRadius)) aOccuInflateRadius = 0.5;
    aNamespace = ros::this_node::getNamespace();

    aHasPose = false;
    aHasOcc = false;
    aReceivedComm = false;

    // initialize communication containers
    aLidarsArray.assign(aLidarSources, geometry_msgs::PoseArray());
    aTrajectoriesArray.assign(aRobots, geometry_msgs::PoseArray());

    // Subscribers
    std::vector<geometry_msgs::PoseArray>* lidarArrPtr = &aLidarsArray;
    for(int lidar_source = 0; lidar_source < aLidarSources; ++lidar_source) {
        aSubscribers.push_back(node_handle.subscribe<geometry_msgs::PoseArray>(aNamespace + "/laser_to_world/lidar_occ_" + std::to_string(lidar_source), aQueueSize,
            [lidarArrPtr, lidar_source](geometry_msgs::PoseArray::ConstPtr msg){
                lidarArrPtr->at(lidar_source).header = msg->header;
                lidarArrPtr->at(lidar_source).poses.assign(msg->poses.begin(), msg->poses.end());
            }
        ));
    }

    std::vector<geometry_msgs::PoseArray>* poseArraysPtr = &aTrajectoriesArray;
    for(int robot = 0; robot < aRobots; ++robot) {
        if(robot <= aId) continue;
        aSubscribers.push_back(node_handle.subscribe<geometry_msgs::PoseArray>("/robot_" + std::to_string(robot) + "/local_planner/optimal_poses", aQueueSize,
            [poseArraysPtr, robot](geometry_msgs::PoseArray::ConstPtr msg){
                poseArraysPtr->at(robot).poses.assign(msg->poses.begin(), msg->poses.end());
            }
        ));
    }

    aSubscribers.push_back(node_handle.subscribe<std_msgs::Int8MultiArray>(
                            aNamespace + "/mock_communication_model/robots_in_comm", 
                            aQueueSize,
                            std::bind(&LocalCSpaceNode::RobotsInCommCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>(
                                aNamespace + "/gmapping_pose/world_pose", 
                                aQueueSize,
                                std::bind(&LocalCSpaceNode::WorldPoseCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(node_handle.subscribe<nav_msgs::OccupancyGrid>(
                            aNamespace + "/map", 
                            aQueueSize, 
                            std::bind(&LocalCSpaceNode::OccCallback, this, std::placeholders::_1)));

    // Advertisers
    aLocalCSpacePublisher = node_handle.advertise<nav_msgs::OccupancyGrid>(aNamespace + "/c_space_local", aQueueSize);
    aOccupiedPositionsPublisher = node_handle.advertise<geometry_msgs::PoseArray>(aNamespace + "/local_occupied_poses", aQueueSize);
    aFreePositionsPublisher = node_handle.advertise<geometry_msgs::PoseArray>(aNamespace + "/local_free_poses", aQueueSize);

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&LocalCSpaceNode::Update, this)));
}

LocalCSpaceNode::~LocalCSpaceNode() {

}

void LocalCSpaceNode::RobotsInCommCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    if(!aReceivedComm) aReceivedComm = true;
    aRobotsInCommMsg.data.assign(msg->data.begin(), msg->data.end());
}

void LocalCSpaceNode::WorldPoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    if(!aHasPose) aHasPose = true;
    aWorldPoseMsg.robot_id = msg->robot_id;
    aWorldPoseMsg.pose.position = msg->pose.position;
    aWorldPoseMsg.pose.orientation = msg->pose.orientation;
}

void LocalCSpaceNode::OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    if(!aHasOcc) aHasOcc = true;
    aOccMsg.data.assign(msg->data.begin(), msg->data.end());
    aOccMsg.info = msg->info;
    aOccMsg.header = msg->header;
}

void LocalCSpaceNode::ClearLocalTrajectories(std::vector<geometry_msgs::PoseArray>& local, std_msgs::Int8MultiArray& comm) {
    for(size_t robot = 0; robot < local.size(); ++robot) {
        if(comm.data[robot] == 0) local[robot].poses.clear();
    }
}

void LocalCSpaceNode::CreateLocal(nav_msgs::OccupancyGrid& dynamicOcc, 
                nav_msgs::OccupancyGrid& localMap,
                geometry_msgs::PoseArray& occupiedPoses,
                geometry_msgs::PoseArray& freePoses,
                tf::Vector3& worldPose,
                tf::Vector3& occPose,
                const double& freeInflationRadius,
                const double& occupiedInflationRadius, 
                const int& windws_size_meters,
                const int8_t& occupancyThreshold,
                const int8_t& freeThreshold,
                const int8_t& occupiedValue,
                const int8_t& freeVal,
                const int8_t& unknownVal) {
    // convert local view size here and compute the size 
    // of the local view
    int size_in_pixels = static_cast<int>(static_cast<float>(windws_size_meters) / dynamicOcc.info.resolution) + 1;
    int length = size_in_pixels + 1;
    int half_length = static_cast<int>(static_cast<float>(length)/2.0);

    Vec2i start = Vec2i::Create(occPose.getX() - half_length, occPose.getY() - half_length);
    Vec2i end   = Vec2i::Create(occPose.getX() + half_length, occPose.getY() + half_length);

    int ix, iy, index;
    int irp_occu = static_cast<int>(occupiedInflationRadius / dynamicOcc.info.resolution);
    int irp_free = static_cast<int>(freeInflationRadius / dynamicOcc.info.resolution);

    // allocate data for the local map
    localMap.data.assign(length*length, -1);
    localMap.header = dynamicOcc.header;
    localMap.info = dynamicOcc.info;
    localMap.info.width = length;
    localMap.info.height = length;
    localMap.info.origin.position.x = worldPose.getX() - (localMap.info.width * dynamicOcc.info.resolution) / 2.0;
    localMap.info.origin.position.y = worldPose.getY() - (localMap.info.height * dynamicOcc.info.resolution) / 2.0;

    nav_msgs::OccupancyGrid free;
    nav_msgs::OccupancyGrid occu;
    free.data.assign(length*length, -1);
    occu.data.assign(length*length, -1);

    // copy local
    int id = 0;
    for(int y = 0; y < length; ++y) {
        iy = y + start.y;
        for(int x = 0; x < length; ++x) {
            ix = x + start.x;
            index = iy * dynamicOcc.info.width + ix;

            // free
            if(dynamicOcc.data[index] >= 0 && dynamicOcc.data[index] <= 50)
                ApplyMask(x,y,irp_free, free.data, freeVal, length, length);
            if(dynamicOcc.data[index] > occupancyThreshold) {
                ApplyMask(x,y,irp_occu, occu.data, occupiedValue, length, length);
                id += 1;
            }
        }
    }

    // inflate and check obstacles polygons' positions
    // get valid obstacles and free pos
    occupiedPoses.poses.clear();
    freePoses.poses.clear();
    ApplyMask(half_length,half_length, 3, free.data, freeVal, length, length);
    localMap.data.assign(free.data.begin(), free.data.end());

    for(int y = 0; y < length; ++y) {
        iy = y + start.y;
        for(int x = 0; x < length; ++x) {
            ix = x + start.x;
            int index = y*length+x;
            if(occu.data[index] > occupancyThreshold) {
                localMap.data[index] = occupiedValue;                
                // set pose array message wih the obstacle's position
                geometry_msgs::Pose p; p.position.x = x; p.position.y = y;
                occupiedPoses.poses.push_back(p);
            } else if (localMap.data[index] >= 0 && localMap.data[index] < occupiedValue) {
                geometry_msgs::Pose p; p.position.x = x; p.position.y = y;
                freePoses.poses.push_back(p);
            }
        }
    }
}

void LocalCSpaceNode::ApplyDynamicData(nav_msgs::OccupancyGrid& occ,
                                         nav_msgs::OccupancyGrid& dynamicOcc,
                                         std::vector<geometry_msgs::PoseArray>& lidarSources,
                                         std::vector<geometry_msgs::PoseArray>& otherSources,
                                         const double& maxLidarRange,
                                         const int8_t& occupiedValue) {
    dynamicOcc.data.assign(occ.data.begin(), occ.data.end());
    dynamicOcc.header = occ.header;
    dynamicOcc.info = occ.info;
    int width = occ.info.width;
    double range;
    Vec2i pos;

    // enhance with peripheral lidars
    for(size_t source = 0; source < lidarSources.size(); ++source) {
        for(size_t pose = 0; pose < lidarSources[source].poses.size(); ++pose) {
            pos.x = static_cast<int>(lidarSources[source].poses[pose].position.x);
            pos.y = static_cast<int>(lidarSources[source].poses[pose].position.y);
            range = lidarSources[source].poses[pose].position.z;
            if(range <= maxLidarRange & range >= 0.01) { 
                dynamicOcc.data[pos.y*width+pos.x] = occupiedValue;
            }
        }
    }

    for(size_t source = 0; source < otherSources.size(); ++source) {
        for(size_t pose = 0; pose < otherSources[source].poses.size(); ++pose) {
            WorldToMap(occ, otherSources[source].poses[pose], pos);
            dynamicOcc.data[pos.y*width+pos.x] = occupiedValue;
        }
    }
}

void LocalCSpaceNode::Update() {
    if(!aHasOcc || !aHasPose || !aReceivedComm) return;

    /*
     * Clear dynamic trajectories in local map
     */
    ClearLocalTrajectories(aTrajectoriesArray, aRobotsInCommMsg);

    ApplyDynamicData(aOccMsg, aOccWithDynamicDataMsg, aLidarsArray, aTrajectoriesArray);
    WorldToMap(aOccMsg, aWorldPoseMsg.pose, aOccPose);

    tf::Vector3 world_pose;
    world_pose.setX(aWorldPoseMsg.pose.position.x);
    world_pose.setY(aWorldPoseMsg.pose.position.y);
    world_pose.setZ(aWorldPoseMsg.pose.position.z);
    
    CreateLocal(aOccWithDynamicDataMsg, 
                aLocalCspaceMsg, 
                aOccupiedPosesMsg, 
                aFreePosesMsg, 
                world_pose,
                aOccPose,
                aFreeInflateRadius, 
                aOccuInflateRadius, 
                aLocalViewSize);

    aLocalCSpacePublisher.publish(aLocalCspaceMsg);
    aOccupiedPositionsPublisher.publish(aOccupiedPosesMsg);
    aFreePositionsPublisher.publish(aFreePosesMsg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "cspacenode");
    std::unique_ptr<LocalCSpaceNode> relativePoseEstimatorNode = std::make_unique<LocalCSpaceNode>();
    ros::spin();
}