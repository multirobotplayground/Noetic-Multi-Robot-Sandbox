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


#include "GlobalMapStitchingNode.h"

GlobalMapStitchingNode::GlobalMapStitchingNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("robots", aRobots)) throw std::runtime_error("Could not retrieve /robots.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    aNamespace = ros::this_node::getNamespace();

    aReceivedRelativePoses = false;
    aDirty = true;
    aReset = false;
    aGlobalPause = false;

    // containers initialization
    aRobotsInCommMsg.data.assign(aRobots, 0);
    aRobotsOcc.assign(aRobots, nav_msgs::OccupancyGrid());  
    aReceivedOccs.assign(aRobots, false);

    // subscriptions
    for(int robot = 0; robot < aRobots; ++robot) {
        nav_msgs::OccupancyGrid* occPtr = &aRobotsOcc[robot];
        std_msgs::Int8MultiArray* commPtr = &aRobotsInCommMsg;
        std::vector<bool>* receivedFlagPtr = &aReceivedOccs;
        nav_msgs::OccupancyGrid* fusionMsg = &aFusionMsg;
        bool* dirty_ptr = &aDirty;
        aSubscribers.push_back(
            node_handle.subscribe<nav_msgs::OccupancyGrid>(
                    "/robot_" + std::to_string(robot) + "/map", 
                    aQueueSize, 
                    [occPtr, receivedFlagPtr, commPtr, fusionMsg, dirty_ptr, robot](nav_msgs::OccupancyGrid::ConstPtr msg){
                        // otherwise, update the last received map
                        occPtr->data.assign(msg->data.begin(), msg->data.end());
                        occPtr->info = msg->info;
                        occPtr->header = msg->header;

                        // initialize first occupancy grid AFTER updating occPtr
                        if(*dirty_ptr) {
                            fusionMsg->info = msg->info;
                            fusionMsg->header = msg->header;
                            fusionMsg->data.assign(msg->data.size(), -1);
                            *dirty_ptr = false;
                        }

                        // set received flag to true
                        receivedFlagPtr->at(robot) = true;
                    }));
    }

    // advertisers
    std::string fusion_topic;
    aFusionPublisher = node_handle.advertise<nav_msgs::OccupancyGrid>("/global_fusion_statistics", aQueueSize);
    
    // Debug: Print the actual topic name being advertised
    ROS_INFO("GlobalMapStitchingNode: Advertising fusion topic at: %s", aFusionPublisher.getTopic().c_str());

    // services
    aResetService = node_handle.advertiseService("/global_fusion/reset", &GlobalMapStitchingNode::ResetCallback, this);

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Bool>(
            "/global_pause",
            aQueueSize,
            std::bind(&GlobalMapStitchingNode::globalPauseCallback, this, std::placeholders::_1)));

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&GlobalMapStitchingNode::Update, this)));
}

GlobalMapStitchingNode::~GlobalMapStitchingNode() {

}

void GlobalMapStitchingNode::globalPauseCallback(std_msgs::Bool::ConstPtr msg) {
    aGlobalPause = msg->data;
    if(aGlobalPause) {
        ROS_WARN("[GlobalMapStitchingNode] Global pause activated. Halting operations.");
    } else {
        ROS_WARN("[GlobalMapStitchingNode] Global pause deactivated. Resuming operations.");
    }
}

void GlobalMapStitchingNode::CommunicationsCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    if(aGlobalPause) return;
    aRobotsInCommMsg.data.assign(msg->data.begin(), msg->data.end());
}

bool GlobalMapStitchingNode::ResetCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("[GlobalMapStitchingNode] Starting global fusion reset");
    
    aReset = true;
    return true;
}

void GlobalMapStitchingNode::set_value(nav_msgs::OccupancyGrid& grid, const int& x, const int& y, int8_t value) {
    if (x >= 0 && x < (int)grid.info.width &&
        y >= 0 && y < (int)grid.info.height) {
            int8_t& to_replace = grid.data[y * grid.info.width + x];
            if(to_replace == -1) to_replace = value;
    }
}

nav_msgs::OccupancyGrid GlobalMapStitchingNode::Stitch(nav_msgs::OccupancyGrid& A, nav_msgs::OccupancyGrid& B) {      
    nav_msgs::OccupancyGrid out;

    // Calculate combined bounds
    double new_origin_x = std::min(A.info.origin.position.x, B.info.origin.position.x);
    double new_origin_y = std::min(A.info.origin.position.y, B.info.origin.position.y);
    double max_x = std::max(A.info.origin.position.x + A.info.width * A.info.resolution,
                        B.info.origin.position.x + B.info.width * B.info.resolution);
    double max_y = std::max(A.info.origin.position.y + A.info.height * A.info.resolution,
                        B.info.origin.position.y + B.info.height * B.info.resolution);

    
    // Set up output grid
    out.header.stamp = ros::Time::now();
    out.header.frame_id = aRobotsOcc[aId].header.frame_id;
    out.info.resolution = aRobotsOcc[aId].info.resolution;

    int new_width = std::round((max_x - new_origin_x) / out.info.resolution);
    int new_height = std::round((max_y - new_origin_y) / out.info.resolution);
    out.info.width = new_width;
    out.info.height = new_height;
    out.info.origin.position.x = new_origin_x;
    out.info.origin.position.y = new_origin_y;
    out.info.origin.position.z = 0.0;
    out.info.origin.orientation.w = 1.0;
    
    // Initialize output data with unknown values (-1)
    out.data.assign(out.info.width * out.info.height, -1);
    
    // Copy data from grid A
    int start_x = (A.info.origin.position.x - new_origin_x) / out.info.resolution;
    int start_y = (A.info.origin.position.y - new_origin_y) / out.info.resolution;

    for (uint32_t y = 0; y < A.info.height; ++y) {
        for (uint32_t x = 0; x < A.info.width; ++x) {                  
            int8_t value = A.data[y * A.info.width + x];
            set_value(out, start_x + x, start_y + y, value);
        }
    }
    
    // Copy data from grid B (with conflict resolution)
    start_x = (B.info.origin.position.x - new_origin_x) / out.info.resolution;
    start_y = (B.info.origin.position.y - new_origin_y) / out.info.resolution;

    for (uint32_t y = 0; y < B.info.height; ++y) {
        for (uint32_t x = 0; x < B.info.width; ++x) {
            int8_t value = B.data[y * B.info.width + x];
            set_value(out, start_x + x, start_y + y, value);
        }
    }

    return out;
}

void GlobalMapStitchingNode::Update() {
    if(aReset) {        
        aReceivedRelativePoses = false;
        aDirty = true;
        aReset = false;

        // containers initialization
        aRobotsInCommMsg.data.assign(aRobots, 0);
        aRobotsOcc.assign(aRobots, nav_msgs::OccupancyGrid());  
        aReceivedOccs.assign(aRobots, false);

        // Clear fusion message completely
        aFusionMsg.data.clear();
        aFusionMsg.data.shrink_to_fit(); // Free memory
        aFusionMsg.info = nav_msgs::MapMetaData();
        aFusionMsg.header = std_msgs::Header();
        aFusionMsg.header.stamp = ros::Time::now();
        aFusionMsg.header.frame_id = "/map";
        
        // Reset all robot occupancy grids
        for(size_t i = 0; i < aRobotsOcc.size(); ++i) {
            aRobotsOcc[i].data.clear();
            aRobotsOcc[i].data.shrink_to_fit(); // Free memory
            aRobotsOcc[i].info = nav_msgs::MapMetaData();
            aRobotsOcc[i].header = std_msgs::Header();
            aRobotsOcc[i].header.stamp = ros::Time::now();
            aRobotsOcc[i].header.frame_id = "robot_" + std::to_string(i) + "/map";
        }
        
        // Reset communication state
        aRobotsInCommMsg.data.assign(aRobots, 0);
        
        // Publish empty fusion map to clear subscribers
        aFusionPublisher.publish(aFusionMsg);
        
        ROS_INFO("[GlobalMapStitchingNode] Global fusion reset completed");

        aReset = false;
    } else {
        if(aGlobalPause) return;
        
        // Check if fusion message is initialized
        if(aReceivedOccs[0] == false) {
            ROS_WARN_THROTTLE(1, "Fusion message not initialized yet");
            return;
        }

        
        // do the map transformation here
        // other adjustments can be made
        // but for the purpose of several experiments,
        // initial translation should be enough
        bool init = false;
        for(int robot = 0; robot < aRobots; ++robot) {
            // do not process my map
            if(aReceivedOccs[0] == false) continue;

            if(!init) {
                aFusionMsg = aRobotsOcc[robot];
                init = true;
                continue;
            }

            aFusionMsg = Stitch(aFusionMsg, aRobotsOcc[robot]);
        }

        // publish fusion, this robot's map will always be on top
        aFusionPublisher.publish(aFusionMsg);
        
        // Debug: Log publishing activity
        ROS_DEBUG_THROTTLE(1, "Published fusion map with %zu data points", aFusionMsg.data.size());
    }
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "globalmapstitchingnode");
    std::unique_ptr<GlobalMapStitchingNode> mapStitchingNode = std::make_unique<GlobalMapStitchingNode>();
    ros::spin();
}
