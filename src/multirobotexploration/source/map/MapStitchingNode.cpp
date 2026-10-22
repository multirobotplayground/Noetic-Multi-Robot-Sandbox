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


#include "MapStitchingNode.h"
#include <std_srvs/Empty.h>

MapStitchingNode::MapStitchingNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots.");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("merge", aMerge)) aMerge = false;
    aNamespace = ros::this_node::getNamespace();

    ROS_INFO(aMerge ? "[MapStitchingNode] Merging enabled" : "[MapStitchingNode] Merging disabled");

    aReceivedRelativePoses = false;
    aDirty = true;
    aInitialized = false;
    aReset = false;
    aGlobalPause = false;

    // containers initialization
    aRobotsInCommMsg.data.assign(aRobots, 0);
    aPrevRobotsInCommMsg.data.assign(aRobots, 0);
    aRobotsOcc.assign(aRobots, nav_msgs::OccupancyGrid());  
    aDirtyArray.assign(aRobots, true);
    aReceivedOccs.assign(aRobots, false);

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
    
    aSubscribers.push_back(
                node_handle.subscribe<std_msgs::Bool>(
                    "/global_pause",
                    aQueueSize,
                    std::bind(&MapStitchingNode::globalPauseCallback, this, std::placeholders::_1)));

    for(int i = 0; i < aRobots; ++i) {
        // do not process my map
        if(i == aId) continue;

        std::vector<nav_msgs::OccupancyGrid>* aRobotsOccPtr = &aRobotsOcc;
        std::vector<bool>* receivedFlagPtr = &aReceivedOccs;
        // subscribe to other robots' maps
        aSubscribers.push_back(
            node_handle.subscribe<nav_msgs::OccupancyGrid>(
                aNamespace + "/robot_" + std::to_string(i) + "/fusion", 
                aQueueSize,
                [this, i, receivedFlagPtr, aRobotsOccPtr](nav_msgs::OccupancyGrid::ConstPtr msg) {
                    // update the last received map
                    (*aRobotsOccPtr)[i].data.assign(msg->data.begin(), msg->data.end());
                    (*aRobotsOccPtr)[i].info = msg->info;
                    (*aRobotsOccPtr)[i].header = msg->header;
                    (*receivedFlagPtr)[i] = true;
                }));
    }

    ros::service::waitForService(aNamespace + "/octomap/reset");

    // advertisers
    aFusionPublisher = node_handle.advertise<nav_msgs::OccupancyGrid>(aNamespace + "/fusion", aQueueSize);
    aResetMapServiceClient = node_handle.serviceClient<std_srvs::Empty>(aNamespace + "/octomap/reset");
    aResetMapService = node_handle.advertiseService(aNamespace + "/reset_map", &MapStitchingNode::ResetMapCallback, this);

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&MapStitchingNode::Update, this)));
}

MapStitchingNode::~MapStitchingNode() {

}

void MapStitchingNode::globalPauseCallback(std_msgs::Bool::ConstPtr msg) {
    aGlobalPause = msg->data;
    if(aGlobalPause) {
        ROS_WARN("[MapStitchingNode] Global pause activated. Halting operations.");
    } else {
        ROS_WARN("[MapStitchingNode] Global pause deactivated. Resuming operations.");
    }
}

bool MapStitchingNode::ResetMapCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
    ROS_INFO("[MapStitchingNode] Starting comprehensive map reset for robot %d", aId);
    aReset = true;
    return true;
}

void MapStitchingNode::CommunicationsCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    if(aGlobalPause) return;

    // Store previous state before updating
    aPrevRobotsInCommMsg = aRobotsInCommMsg;
    
    // Update current communication state
    aRobotsInCommMsg.data.assign(msg->data.begin(), msg->data.end());
    
    // // Check for robots that just came into communication (edge detection)
    for(size_t i = 0; i < std::min(aRobots, (int)aRobotsInCommMsg.data.size()); ++i) {
        if(aRobotsInCommMsg.data[i] == 1 && aPrevRobotsInCommMsg.data[i] == 0) {
            CallOccService(i);
            aDirtyArray[i] = true;
            ROS_DEBUG("[MapStitchingNode] Robot %zu came into communication, marking as dirty", i);
        }
    }
}

void MapStitchingNode::CallOccService(const int& robotId) {
    nav_msgs::GetMap::Request req;
    nav_msgs::GetMap::Response res;

    ros::ServiceClient client = ros::NodeHandle().serviceClient<nav_msgs::GetMap>(
        "/robot_" + std::to_string(robotId) + "/service_request_map");

    // Call the service
    if (client.call(req, res)) {
        

        // If successful, update the occupancy grid
        aRobotsOcc[robotId].data.assign(res.map.data.begin(), res.map.data.end());
        aRobotsOcc[robotId].info = res.map.info;
        aRobotsOcc[robotId].header = res.map.header;
        ROS_DEBUG("[MapStitchingNode] Occupancy grid for robot %d updated successfully", robotId);
        aReceivedOccs[robotId] = true;
    } else {
        ROS_ERROR("[MapStitchingNode] Failed to call occupancy grid service for robot %d", robotId);
    }
}

void MapStitchingNode::OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    if(aGlobalPause) return;
    aRobotsOcc[aId].data.assign(msg->data.begin(), msg->data.end());
    aRobotsOcc[aId].info = msg->info;
    aRobotsOcc[aId].header = msg->header;
    aReceivedOccs[aId] = true;
}

void MapStitchingNode::set_value(nav_msgs::OccupancyGrid& grid, const int& x, const int& y, int8_t value, const bool& copy) {
    if (x >= 0 && x < (int)grid.info.width &&
        y >= 0 && y < (int)grid.info.height) {
            int8_t& to_replace = grid.data[y * grid.info.width + x];
            if(to_replace == -1) {
                to_replace = value;
            } else {
                if(value != -1) {
                    to_replace = value;
                }
            }
    }
}

nav_msgs::OccupancyGrid MapStitchingNode::Stitch(nav_msgs::OccupancyGrid& A, nav_msgs::OccupancyGrid& B, const bool& copy) {      
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

    int new_width = (max_x - new_origin_x) / out.info.resolution;
    int new_height = (max_y - new_origin_y) / out.info.resolution;
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
            set_value(out, start_x + x, start_y + y, value, copy);
        }
    }
    
    // Copy data from grid B (with conflict resolution)
    start_x = (B.info.origin.position.x - new_origin_x) / out.info.resolution;
    start_y = (B.info.origin.position.y - new_origin_y) / out.info.resolution;

    for (uint32_t y = 0; y < B.info.height; ++y) {
        for (uint32_t x = 0; x < B.info.width; ++x) {
            int8_t value = B.data[y * B.info.width + x];
            set_value(out, start_x + x, start_y + y, value, copy);
        }
    }

    return out;
}

void MapStitchingNode::Update() {
    if(aReset) {
        // Step 1: Reset underlying octomap service with retry mechanism
        bool octomap_reset_success = false;
        const int max_octomap_retries = 3;
        const double octomap_retry_delay = 0.3;
        std_srvs::Empty::Request req;
        std_srvs::Empty::Response res;
        for (int attempt = 0; attempt < max_octomap_retries && !octomap_reset_success; ++attempt) {
            ROS_INFO("[MapStitchingNode] Octomap reset attempt %d/%d", attempt + 1, max_octomap_retries);
            
            if (aResetMapServiceClient.call(req, res)) {
                ROS_INFO("[MapStitchingNode] Octomap reset successful on attempt %d", attempt + 1);
                octomap_reset_success = true;
            } else {
                ROS_WARN("[MapStitchingNode] Octomap reset failed, attempt %d", attempt + 1);
                if (attempt < max_octomap_retries - 1) {
                    ros::Duration(octomap_retry_delay).sleep();
                }
            }
        }
        
        if (!octomap_reset_success) {
            ROS_ERROR("[MapStitchingNode] Failed to reset octomap after %d attempts", max_octomap_retries);
        }
        
        aReceivedRelativePoses = false;
        aDirty = true;
        aInitialized = false;
        aRobotsInCommMsg.data.assign(aRobots, 0);
        aPrevRobotsInCommMsg.data.assign(aRobots, 0);
        aRobotsOcc.assign(aRobots, nav_msgs::OccupancyGrid());  
        aDirtyArray.assign(aRobots, true);
        aReceivedOccs.assign(aRobots, false);
        
        aFusionMsg.data.clear();
        aFusionMsg.data.shrink_to_fit(); // Free memory
        aFusionMsg.info = nav_msgs::MapMetaData();
        aFusionMsg.header = std_msgs::Header();
        aFusionMsg.header.stamp = ros::Time::now();
        aFusionMsg.header.frame_id = "robot_" + std::to_string(aId) + "/map";
        
        for(size_t i = 0; i < aRobotsOcc.size(); ++i) {
            aRobotsOcc[i].data.clear();
            aRobotsOcc[i].data.shrink_to_fit(); // Free memory
            aRobotsOcc[i].info = nav_msgs::MapMetaData();
            aRobotsOcc[i].header = std_msgs::Header();
            aRobotsOcc[i].header.stamp = ros::Time::now();
            aRobotsOcc[i].header.frame_id = "robot_" + std::to_string(i) + "/map";
        }
        
        if (octomap_reset_success) {
            ROS_INFO("[MapStitchingNode] Complete map reset successful");
        } else {
            ROS_WARN("[MapStitchingNode] Partial map reset - internal data cleared but octomap reset failed");
        }

        aReset = false;
    } else {
        if(aGlobalPause) return;
        
        if(aReceivedOccs[aId] == false) return;

        // do the map transformation here
        // other adjustments can be made
        // but for the purpose of several experiments,
        // initial translation should be enough
        if(aMerge) {
            // initialize first occupancy grid
            aFusionMsg.info = aRobotsOcc[aId].info;
            aFusionMsg.header = aRobotsOcc[aId].header;
            aFusionMsg.data.assign(aRobotsOcc[aId].data.size(), -1);

            for(int robot = 0; robot < aRobots; ++robot) {
                // do not process my map
                if(aReceivedOccs[robot] == false || robot == aId) continue;
                aFusionMsg = Stitch(aFusionMsg, aRobotsOcc[robot], false);
            }

            // copy my map on top of everyone
            aFusionMsg = Stitch(aFusionMsg, aRobotsOcc[aId], true);   

            // publish fusion, this robot's map will always be on top
            aFusionPublisher.publish(aFusionMsg);
        } else {
            aFusionPublisher.publish(aRobotsOcc[aId]); 
        }
    }
}

void MapStitchingNode::RelativeStartingPosesCallback(geometry_msgs::PoseArray::ConstPtr msg) {
    // Store the relative starting poses for map alignment
    // This callback handles relative pose information for proper map stitching
    aReceivedRelativePoses = true;
    aRobotsRelativePosesMsg.header = msg->header;
    aRobotsRelativePosesMsg.poses = msg->poses;
    ROS_DEBUG("[MapStitchingNode] Received relative starting poses with %zu poses", msg->poses.size());
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "mapstitchingnode");
    std::unique_ptr<MapStitchingNode> mapStitchingNode = std::make_unique<MapStitchingNode>();
    ros::spin();
}
