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

#include "CSpaceNode.h"

CSpaceNode::CSpaceNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("lidar_sources", aLidarSources)) throw std::runtime_error("Could not retrieve lidar_sources.");
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
                            std::bind(&CSpaceNode::RobotsInCommCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>(
                                aNamespace + "/gmapping_pose/world_pose", 
                                aQueueSize,
                                std::bind(&CSpaceNode::WorldPoseCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(node_handle.subscribe<nav_msgs::OccupancyGrid>(
                            aNamespace + "/map", 
                            aQueueSize, 
                            std::bind(&CSpaceNode::OccCallback, this, std::placeholders::_1)));

    // Advertisers
    aCspacePublisher = node_handle.advertise<nav_msgs::OccupancyGrid>(aNamespace + "/c_space", aQueueSize);

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&CSpaceNode::Update, this)));
}

CSpaceNode::~CSpaceNode() {

}

void CSpaceNode::RobotsInCommCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    if(!aReceivedComm) aReceivedComm = true;
    aRobotsInCommMsg.data.assign(msg->data.begin(), msg->data.end());
}

void CSpaceNode::WorldPoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    if(!aHasPose) aHasPose = true;
    aWorldPoseMsg.robot_id = msg->robot_id;
    aWorldPoseMsg.pose.position = msg->pose.position;
    aWorldPoseMsg.pose.orientation = msg->pose.orientation;
}

void CSpaceNode::OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    if(!aHasOcc) aHasOcc = true;
    aOccMsg.data.assign(msg->data.begin(), msg->data.end());
    aOccMsg.info = msg->info;
    aOccMsg.header = msg->header;
}

void CSpaceNode::Inflate(nav_msgs::OccupancyGrid& occ,
            nav_msgs::OccupancyGrid& free,
            nav_msgs::OccupancyGrid& occupied, 
            const double& freeInflationRadius,
            const double& occupiedInflationRadius, 
            const int8_t& occupancyThreshold,
            const int8_t& freeThreshold,
            const int8_t& occupiedValue,
            const int8_t& freeVal) {
    free.data.assign(occ.data.begin(), occ.data.end());
    free.header = occ.header;
    free.info = occ.info;

    occupied.data.assign(occ.data.begin(), occ.data.end());
    occupied.header = occ.header;
    occupied.info = occ.info;

    int index;
    int8_t val;
    int8_t raw_val;
    int width = occ.info.width;
    int height = occ.info.height;
    int irp_occu = static_cast<int>(occupiedInflationRadius / occ.info.resolution);
    int irp_free = static_cast<int>(freeInflationRadius / occ.info.resolution);
    for(int y = 0; y < occ.info.height; ++y) {
        for(int x = 0; x < occ.info.width; ++x) {
            index = y * width + x;
            val = occ.data[index];
            if(val > occupancyThreshold)
                ApplyMask(x, y, irp_occu, occupied.data, occupiedValue, width, height);
            else if(val >= 0 && val < freeThreshold)
                ApplyMask(x, y, irp_free, free.data, freeVal, width, height);
        }
    }
}

void CSpaceNode::ApplyDynamicData(nav_msgs::OccupancyGrid& occ,
                                         nav_msgs::OccupancyGrid& dynamicOcc,
                                         std::vector<geometry_msgs::PoseArray>& lidarSources,
                                         const double& maxLidarRange,
                                         const int8_t& occupiedValue) {
    dynamicOcc.data.assign(occ.data.begin(), occ.data.end());
    dynamicOcc.header = occ.header;
    dynamicOcc.info = occ.info;
    double range;
    int width = occ.info.width;
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
}

void CSpaceNode::GenerateCSpace(nav_msgs::OccupancyGrid& free,
                    nav_msgs::OccupancyGrid& occupied,
                    nav_msgs::OccupancyGrid& cspace,
                    tf::Vector3& occ_pose,
                    const int8_t& unknownVal) {
    // copy data for local grids
    cspace.data.assign(occupied.data.begin(), occupied.data.end());
    cspace.header = occupied.header;
    cspace.info = occupied.info;

    int occ_val;
    int fre_val;
    int x = occ_pose.getX();
    int y = occ_pose.getY();
    
    for(size_t index = 0; index < cspace.data.size(); ++index) {
        occ_val = occupied.data[index];
        fre_val = free.data[index];

        // should cpy the filtered free values and then the occupied ones
        // this creates a map without noise
        if(fre_val != unknownVal) cspace.data[index] = fre_val;
        if(occ_val != unknownVal) cspace.data[index] = occ_val;
    }
}

void CSpaceNode::InflatePoseForPlanner(nav_msgs::OccupancyGrid& cspace,
                           const double& freeInflationRadius,
                           const int& x, 
                           const int& y,
                           const int8_t& occupancyThreshold,
                           const int8_t& freeVal) {
    int irp_free = static_cast<int>(freeInflationRadius / cspace.info.resolution);
    ApplyMask(x,y,irp_free,cspace.data,freeVal,cspace.info.width,cspace.info.height,occupancyThreshold,false);
}

void CSpaceNode::ApplyDynamicData(nav_msgs::OccupancyGrid& occ,
                                         nav_msgs::OccupancyGrid& dynamicOcc,
                                         std::vector<geometry_msgs::PoseArray>& lidarSources,
                                         std::vector<geometry_msgs::PoseArray>& otherSources,
                                         const double& maxLidarRange,
                                         const int8_t& occupiedValue) {
    dynamicOcc.data.assign(occ.data.begin(), occ.data.end());
    dynamicOcc.header = occ.header;
    dynamicOcc.info = occ.info;
    double range;
    int width = occ.info.width;
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

void CSpaceNode::ClearLocalTrajectories(std::vector<geometry_msgs::PoseArray>& local, std_msgs::Int8MultiArray& comm) {
    for(size_t robot = 0; robot < local.size(); ++robot) {
        if(comm.data[robot] == 0) local[robot].poses.clear();
    }
}

void CSpaceNode::Update() {
    if(!aHasOcc || !aHasPose || !aReceivedComm) return;

    /*
     * Clear dynamic trajectories in local map
     */
    ClearLocalTrajectories(aTrajectoriesArray, aRobotsInCommMsg);

    ApplyDynamicData(aOccMsg, aOccWithDynamicDataMsg, aLidarsArray, aTrajectoriesArray);

    // create c_space at the secified rate
    Inflate(aOccWithDynamicDataMsg, aFreeCellsMsg, aOccupiedCellsMsg, aFreeInflateRadius, aOccuInflateRadius);
    
    WorldToMap(aOccMsg, aWorldPoseMsg.pose, aOccPose);
    GenerateCSpace(aFreeCellsMsg, aOccupiedCellsMsg, aCspaceMsg, aOccPose);
    InflatePoseForPlanner(aCspaceMsg, aFreeInflateRadius, aOccPose.getX(), aOccPose.getY());

    // publish the occ
    aCspacePublisher.publish(aCspaceMsg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "cspacenode");
    std::unique_ptr<CSpaceNode> relativePoseEstimatorNode = std::make_unique<CSpaceNode>();
    ros::spin();
}