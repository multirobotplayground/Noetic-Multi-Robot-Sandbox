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

#include "FrontierDiscoveryNode.h"
#include <queue>

/*
 * Node implementation
 */
FrontierDiscoveryNode::FrontierDiscoveryNode() {
    aState = FrontierState::IDLE;

    // load all parameters
    ros::NodeHandle node_handle("~");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve robot id.");
    if(!node_handle.getParam("max_lidar_range", aMaxLidarRange)) aMaxLidarRange = 10.0;
    if(!node_handle.getParam("cluster_detection_min", aClusterDetectionMin)) aClusterDetectionMin = 30;
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    aNamespace = ros::this_node::getNamespace();

    // subscriptions
    aSubscribers.push_back(
        node_handle.subscribe<nav_msgs::OccupancyGrid>(
            aNamespace + "/c_space", 
            aQueueSize, 
            std::bind(&FrontierDiscoveryNode::CSpaceCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::CustomPose>(
            aNamespace + "/world_pose", 
            aQueueSize, 
            std::bind(&FrontierDiscoveryNode::EstimatePoseCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/frontier_discovery/compute", 
            aQueueSize, 
            std::bind(&FrontierDiscoveryNode::ComputeCallback, this, std::placeholders::_1)));

    // advertisers
    aClusterMarkerPub     = node_handle.advertise<visualization_msgs::Marker>(aNamespace + "/frontier_discovery/frontiers_clusters_markers", aQueueSize);
    aFrontiersMapPub      = node_handle.advertise<nav_msgs::OccupancyGrid>(aNamespace + "/frontier_discovery/frontiers", aQueueSize);
    aFrontiersClustersPub = node_handle.advertise<multirobotsimulations::Frontiers>(aNamespace + "/frontier_discovery/frontiers_clusters", aQueueSize);
    aFrontierRequestService = node_handle.advertiseService(aNamespace + "/frontier_discovery/request", &FrontierDiscoveryNode::ServiceRequest, this);

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&FrontierDiscoveryNode::Update, this)));
}

FrontierDiscoveryNode::~FrontierDiscoveryNode() {
    
}

bool FrontierDiscoveryNode::ServiceRequest(multirobotsimulations::frontierservice::Request& req, multirobotsimulations::frontierservice::Response& res) {
    // iterate infinitely 
    while(aReceivedCSpace != true && aHasPose != true) {
        ROS_INFO("[FrontierDiscovery] Waiting for CSpace and Pose...");
    }

    WorldToMap(aOcc, aWorldPos, aPos);
    CheckReachableFrontiers(aOcc, aFrontiersMap, aPos, aFrontiers);
    sa::ComputeClusters(aFrontiersMap, aFrontiers, aClusters);

    aFilteredClusters.clear();
    for(auto& cluster : aClusters) {
        if(cluster.size() > aClusterDetectionMin) aFilteredClusters.push_back(cluster);
    }
    
    sa::ComputeAverageCentroids(aPos, aFilteredClusters, aCentroids);

    // filter reachable
    aFilteredCentroids.clear();
    ResetFrontierMsg(aFrontierMsg);       

    std::list<Vec2i> path;
    double cost, value, utility;
    for(size_t i = 0; i < aCentroids.size(); ++i) {
        // compute convex hull of the frontiers
        //cost = Distance(aPos, aCentroids[i]) * static_cast<double>(aOcc.info.resolution);
        sa::ComputePath(aOcc, aPos, aCentroids[i], path);
        if(path.size() > 0) {
            cost = path.size() * static_cast<double>(aOcc.info.resolution);
        } else {
            cost = Distance(aPos, aCentroids[i]) * static_cast<double>(aOcc.info.resolution);
        }

        value = ComputeCentroidValue(aOcc, aCentroids[i], aMaxLidarRange);
        utility = value / cost;

        aFilteredCentroids.push_back(aCentroids[i]);
        res.costs.data.push_back(cost);
        res.values.data.push_back(value);
        res.utilities.data.push_back(utility);
    }
    ROS_INFO("[FrontierDiscovery] Found %ld valid centroids.", aFilteredCentroids.size());

    // publish the found frontiers centroids into the network
    if(aFilteredCentroids.size() > 0) {
        SetPoseArr(aPoseArrMsg, aSeq);
        CreateMarker(aClusterMarkerMsg, aNamespace.c_str(), aId, aSeq);

        ROS_INFO("[FrontierDiscovery] %ld available frontiers.", aFilteredCentroids.size());
        for(size_t i = 0; i < aFilteredCentroids.size(); ++i) {
            // hook utility
            if(res.utilities.data[i] > res.highest_utility) {
                res.highest_utility = res.utilities.data[i];
                res.highest_utility_index = static_cast<uint8_t>(i);
            }
            if(res.utilities.data[i] <= res.lowest_utility) {
                res.lowest_utility = res.utilities.data[i];
                res.lowest_utility_index = static_cast<uint8_t>(i);
            }

            // hook cost
            if(res.costs.data[i] > res.highest_cost) {
                res.highest_cost = res.costs.data[i];
                res.highest_cost_index = static_cast<uint8_t>(i);
            }
            if(res.costs.data[i] <= res.lowest_cost) {
                res.lowest_cost = res.costs.data[i];
                res.lowest_cost_index = static_cast<uint8_t>(i);
            }

            // hook value
            if(res.values.data[i] > res.highest_value) {
                res.highest_value = res.values.data[i];
                res.highest_value_index = static_cast<uint8_t>(i);
            }
            if(res.values.data[i] <= res.lowest_value) {
                res.lowest_value = res.values.data[i];
                res.lowest_value_index = static_cast<uint8_t>(i);
            }

            tf::Vector3 temp_world;
            MapToWorld(aOcc, aFilteredCentroids[i], temp_world);
            geometry_msgs::Point p;
            geometry_msgs::Pose po;
            p.z = 0.25;
            p.x = temp_world.getX();
            p.y = temp_world.getY();
            po.position.x = temp_world.getX();
            po.position.y = temp_world.getY();
            aClusterMarkerMsg.points.push_back(p);
            aPoseArrMsg.poses.push_back(po);

            ROS_INFO("\t[%.2f %.2f] - cost: %.2f value: %.2f utility: %.2f", 
                temp_world.getX(),
                temp_world.getY(),
                res.costs.data[i], 
                res.values.data[i], 
                res.utilities.data[i]);
        }
        res.centroids = aPoseArrMsg;

        aClusterMarkerPub.publish(aClusterMarkerMsg);
        aFrontiersMapPub.publish(aFrontiersMap);

        aSeq += 1;
    }
    
    return true;
}

void FrontierDiscoveryNode::CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    // Validate incoming occupancy grid
    if(msg->info.width == 0 || msg->info.height == 0 || msg->info.resolution <= 0.0) {
        ROS_ERROR("[FrontierDiscovery] Received invalid occupancy grid: width=%d, height=%d, resolution=%f", 
                  msg->info.width, msg->info.height, msg->info.resolution);
        return;
    }

    int expected_size = msg->info.width * msg->info.height;
    if(static_cast<int>(msg->data.size()) != expected_size) {
        ROS_ERROR("[FrontierDiscovery] Received occupancy grid with size mismatch: expected=%d, actual=%zu", 
                  expected_size, msg->data.size());
        return;
    }

    aOcc.data.assign(msg->data.begin(), msg->data.end());
    aOcc.info = msg->info;
    aOcc.header = msg->header;
    aReceivedCSpace = true;
    
    ROS_DEBUG("[FrontierDiscovery] Received valid occupancy grid: %dx%d, resolution=%.3f", 
              aOcc.info.width, aOcc.info.height, aOcc.info.resolution);
}

void FrontierDiscoveryNode::EstimatePoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    aWorldPos.position = msg->pose.position;
    aWorldPos.orientation = msg->pose.orientation;
    aYaw = tf::getYaw(msg->pose.orientation);
    aHasPose = true;
}

void FrontierDiscoveryNode::ComputeCallback(std_msgs::String::ConstPtr msg) {
    ROS_INFO("[FrontierDiscovery] Received request: %s", msg->data.c_str());
    aState = FrontierState::PROCESSING;
}

void FrontierDiscoveryNode::CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq) {
    input.id = id;
    input.header.frame_id = "robot_" + std::to_string(id) + std::string("/map");
    input.header.stamp = ros::Time().now();
    input.ns = ns;
    input.points.clear();
    input.type = visualization_msgs::Marker::CUBE_LIST;
    input.action = visualization_msgs::Marker::MODIFY;
    input.pose.orientation.x = 0.0;
    input.pose.orientation.y = 0.0;
    input.pose.orientation.z = 0.0;
    input.pose.orientation.w = 1.0;
    input.scale.x = 0.5;
    input.scale.y = 0.5;
    input.scale.z = 0.5;
    input.color.a = 1.0;
    input.color.r = 0.0;
    input.color.g = 0.3;
    input.color.b = 1.0;
    input.lifetime = ros::Duration(60);
}

void FrontierDiscoveryNode::SetPoseArr(geometry_msgs::PoseArray& arr, const int& seq) {
    arr.poses.clear();
    arr.header.frame_id = std::string("robot_") + std::to_string(aId) + std::string("/map");
    arr.header.seq = seq;
    arr.header.stamp = ros::Time::now();
}

void FrontierDiscoveryNode::ResetFrontierMsg(multirobotsimulations::Frontiers& msg) {
    msg.centroids.poses.clear();
    msg.costs.data.clear();
    msg.values.data.clear();
    msg.utilities.data.clear();

    // Use these control variables to get max and min values during single loop
    // avoid calling search everytime until having something better
    msg.lowest_utility_index = -1;
    msg.lowest_cost_index = -1;
    msg.lowest_value_index = -1;

    msg.highest_utility_index = -1;
    msg.highest_cost_index = -1;
    msg.highest_value_index = -1;

    msg.lowest_cost = std::numeric_limits<float>::max();
    msg.lowest_value = std::numeric_limits<float>::max();
    msg.lowest_utility = std::numeric_limits<float>::max();
    
    msg.highest_cost = -1.0;
    msg.highest_value = -1.0;
    msg.highest_utility = -1.0;
}

double FrontierDiscoveryNode::ComputeCentroidValue(nav_msgs::OccupancyGrid& occ, Vec2i& centroid, const double& lidarRange) {
    // Validate occupancy grid dimensions
    if(occ.info.width == 0 || occ.info.height == 0 || occ.info.resolution <= 0.0) {
        ROS_WARN("[FrontierDiscovery] Invalid occupancy grid dimensions: width=%d, height=%d, resolution=%f", 
                 occ.info.width, occ.info.height, occ.info.resolution);
        return 0.0;
    }

    double range_squared = lidarRange * lidarRange;
    int range_in_cells = static_cast<int>(lidarRange / occ.info.resolution);

    // Debug information
    ROS_DEBUG("[FrontierDiscovery] Processing centroid (%d, %d) with range %.2f (cells: %d)", 
              centroid.x, centroid.y, lidarRange, range_in_cells);

    Vec2i min = Vec2i::Create(centroid.x-range_in_cells,centroid.y-range_in_cells);
    Vec2i max = Vec2i::Create(centroid.x+range_in_cells,centroid.y+range_in_cells);

    // Debug bounds before clamping
    ROS_DEBUG("[FrontierDiscovery] Initial bounds: min=(%d, %d), max=(%d, %d)", 
              min.x, min.y, max.x, max.y);

    // clamp the ranges
    min.x = std::max(0, min.x);
    min.y = std::max(0, min.y);
    max.x = std::min(max.x, static_cast<int>(occ.info.width));
    max.y = std::min(max.y, static_cast<int>(occ.info.height));

    // Debug bounds after clamping
    ROS_DEBUG("[FrontierDiscovery] Clamped bounds: min=(%d, %d), max=(%d, %d), grid=(%d, %d)", 
              min.x, min.y, max.x, max.y, occ.info.width, occ.info.height);

    // Validate final bounds
    if(min.x >= max.x || min.y >= max.y) {
        ROS_WARN("[FrontierDiscovery] Invalid bounds after clamping: min=(%d, %d), max=(%d, %d)", 
                 min.x, min.y, max.x, max.y);
        return 0.0;
    }

    // count area in cells
    int count = 0;
    for(int x = min.x; x < max.x; ++x) {
        for(int y = min.y; y < max.y; ++y) {
            // Validate x,y coordinates are within bounds before calculating index
            if(x < 0 || x >= static_cast<int>(occ.info.width) || 
               y < 0 || y >= static_cast<int>(occ.info.height)) {
                ROS_WARN("[FrontierDiscovery] Coordinates out of bounds: x=%d, y=%d (width=%d, height=%d)", 
                         x, y, occ.info.width, occ.info.height);
                continue;
            }

            /*
             * Check circle model
             * (x−x1)^2+(y−y1)^2=r^2
             */

            // ensure that the dx and dy are in meters and not in
            // pixels...
            double dx = (centroid.x - x) * occ.info.resolution;
            double dy = (centroid.y - y) * occ.info.resolution;
            double circle_test = dx * dx + dy * dy;

            if(circle_test <= range_squared) {
                int index = y * static_cast<int>(occ.info.width) + x;
                
                // Additional bounds check for the calculated index
                if(index >= 0 && index < static_cast<int>(occ.data.size())) {
                    if(occ.data[index] == -1) {
                        count++;
                    }
                } else {
                    ROS_ERROR("[FrontierDiscovery] Index out of bounds: %d (x=%d, y=%d, width=%d, height=%d, data_size=%zu)", 
                             index, x, y, occ.info.width, occ.info.height, occ.data.size());
                }
            }
        }
    }

    double cell_area = occ.info.resolution * occ.info.resolution;
    double total_area_value = static_cast<double>(count) * cell_area;
    return total_area_value;
}

bool CheckAny(nav_msgs::OccupancyGrid& rInput, const Vec2i& rStart, const Vec2i& rEnd, const int& rVal) {
        Vec2i start = rStart;
        Vec2i end = rEnd;
        int width = rInput.info.width;
        int height = rInput.info.height;
        if(start.x < 0) start.x = 0;
        if(start.x >= width) start.x = width - 1;
        if(start.y < 0) start.y = 0;
        if(start.y >= height) start.y = height - 1;
        if(end.x < 0) end.x = 0;
        if(end.x > width) end.x = width;
        if(end.y < 0) end.y = 0;
        if(end.y > height) end.y = height;

        // this for should be end inclusive
        // thus is the end.[xy] + 1
        for(int y = start.y; y < end.y + 1; ++y) {
            for(int x = start.x; x < end.x + 1; ++x) {
                if(rInput.data[y*width+x] == rVal) return true;
            }
        }
        return false;
    }

void FrontierDiscoveryNode::CheckReachableFrontiers(nav_msgs::OccupancyGrid& occ, nav_msgs::OccupancyGrid& out, const Vec2i& pos, 
std::vector<Vec2i>& reachable_frontiers) {
    reachable_frontiers.clear();
    // Check if the position is within bounds
    if(pos.x < 0 || pos.x >= static_cast<int>(occ.info.width) || 
       pos.y < 0 || pos.y >= static_cast<int>(occ.info.height)) {
        return;
    }

    out.data.assign(occ.data.size(), -2);
    out.info = occ.info;
    out.header = occ.header;

    out.data[pos.y * static_cast<int>(occ.info.width) + pos.x] = -1;
    std::queue<Vec2i> q;
    q.push(pos);
    Vec2i start, end;
    while(!q.empty()) {
        Vec2i current = q.front();
        q.pop();

        for(int dx = -1; dx <= 1; ++dx) {
            for(int dy = -1; dy <= 1; ++dy) {
                if((dx == 0 && dy == 0)) continue; // skip self and diagonals
                Vec2i neighbor = Vec2i::Create(current.x + dx, current.y + dy);
                
                // Check bounds
                if(neighbor.x < 0 || neighbor.x >= static_cast<int>(occ.info.width) || 
                   neighbor.y < 0 || neighbor.y >= static_cast<int>(occ.info.height)) {
                    continue;
                }
                
                start.x = neighbor.x - 1; start.y = neighbor.y - 1;
                end.x   = neighbor.x + 1; end.y   = neighbor.y + 1;
                int neighbor_index = neighbor.y * static_cast<int>(occ.info.width) + neighbor.x;
                if(occ.data[neighbor_index] >= 0 && 
                    occ.data[neighbor_index] < 50 && 
                    out.data[neighbor_index] == -2) {
                    q.push(neighbor);

                    if(CheckAny(occ, start, end, -1)) {
                        reachable_frontiers.push_back(neighbor);
                        out.data[neighbor_index] = 100; // Mark as frontier
                    } else {
                        out.data[neighbor_index] = -1;
                    }
                }
            }
        }
    }
}

void FrontierDiscoveryNode::Update() {
    if(aReceivedCSpace == false || aHasPose == false) return;
    
    double cost, value, utility;
    double min = std::numeric_limits<double>::max();
    int min_index = -1;
    std::list<Vec2i> path;
    switch(aState) {
        case IDLE:
            // do nothing
        break;
        case PROCESSING:
            WorldToMap(aOcc, aWorldPos, aPos);
            CheckReachableFrontiers(aOcc, aFrontiersMap, aPos, aFrontiers);
            sa::ComputeClusters(aFrontiersMap, aFrontiers, aClusters);

            aFilteredClusters.clear();
            for(auto& cluster : aClusters) {
                if(cluster.size() > aClusterDetectionMin) aFilteredClusters.push_back(cluster);
            }
            
            sa::ComputeAverageCentroids(aPos, aFilteredClusters, aCentroids);

            // filter reachable
            aFilteredCentroids.clear();
            ResetFrontierMsg(aFrontierMsg);            

            for(size_t i = 0; i < aCentroids.size(); ++i) {
                // compute convex hull of the frontiers
                //cost = Distance(aPos, aCentroids[i]) * static_cast<double>(aOcc.info.resolution);
                sa::ComputePath(aOcc, aPos, aCentroids[i], path);                
                if(path.size() > 0) {
                    cost = path.size() * static_cast<double>(aOcc.info.resolution);
                } else {
                    cost = Distance(aPos, aCentroids[i]) * static_cast<double>(aOcc.info.resolution);
                }

                value = ComputeCentroidValue(aOcc, aCentroids[i], aMaxLidarRange);
                utility = value / cost;

                aFilteredCentroids.push_back(aCentroids[i]);
                aFrontierMsg.costs.data.push_back(cost);
                aFrontierMsg.values.data.push_back(value);
                aFrontierMsg.utilities.data.push_back(utility);
            }
            ROS_INFO("[FrontierDiscovery] Found %ld valid centroids.", aFilteredCentroids.size());

            // publish the found frontiers centroids into the network
            if(aFilteredCentroids.size() > 0) {
                SetPoseArr(aPoseArrMsg, aSeq);
                CreateMarker(aClusterMarkerMsg, aNamespace.c_str(), aId, aSeq);

                ROS_INFO("[FrontierDiscovery] %ld available frontiers.", aFilteredCentroids.size());
                for(size_t i = 0; i < aFilteredCentroids.size(); ++i) {
                    // hook utility
                    if(aFrontierMsg.utilities.data[i] > aFrontierMsg.highest_utility) {
                        aFrontierMsg.highest_utility = aFrontierMsg.utilities.data[i];
                        aFrontierMsg.highest_utility_index = static_cast<uint8_t>(i);
                    }
                    if(aFrontierMsg.utilities.data[i] <= aFrontierMsg.lowest_utility) {
                        aFrontierMsg.lowest_utility = aFrontierMsg.utilities.data[i];
                        aFrontierMsg.lowest_utility_index = static_cast<uint8_t>(i);
                    }

                    // hook cost
                    if(aFrontierMsg.costs.data[i] > aFrontierMsg.highest_cost) {
                        aFrontierMsg.highest_cost = aFrontierMsg.costs.data[i];
                        aFrontierMsg.highest_cost_index = static_cast<uint8_t>(i);
                    }
                    if(aFrontierMsg.costs.data[i] <= aFrontierMsg.lowest_cost) {
                        aFrontierMsg.lowest_cost = aFrontierMsg.costs.data[i];
                        aFrontierMsg.lowest_cost_index = static_cast<uint8_t>(i);
                    }

                    // hook value
                    if(aFrontierMsg.values.data[i] > aFrontierMsg.highest_value) {
                        aFrontierMsg.highest_value = aFrontierMsg.values.data[i];
                        aFrontierMsg.highest_value_index = static_cast<uint8_t>(i);
                    }
                    if(aFrontierMsg.values.data[i] <= aFrontierMsg.lowest_value) {
                        aFrontierMsg.lowest_value = aFrontierMsg.values.data[i];
                        aFrontierMsg.lowest_value_index = static_cast<uint8_t>(i);
                    }

                    tf::Vector3 temp_world;
                    MapToWorld(aOcc, aFilteredCentroids[i], temp_world);
                    geometry_msgs::Point p;
                    geometry_msgs::Pose po;
                    p.z = 0.25;
                    p.x = temp_world.getX();
                    p.y = temp_world.getY();
                    po.position.x = temp_world.getX();
                    po.position.y = temp_world.getY();
                    aClusterMarkerMsg.points.push_back(p);
                    aPoseArrMsg.poses.push_back(po);

                    ROS_INFO("\t[%.2f %.2f] - cost: %.2f value: %.2f utility: %.2f", 
                        temp_world.getX(),
                        temp_world.getY(),
                        aFrontierMsg.costs.data[i], 
                        aFrontierMsg.values.data[i], 
                        aFrontierMsg.utilities.data[i]);
                }
                aFrontierMsg.centroids = aPoseArrMsg;

                aClusterMarkerPub.publish(aClusterMarkerMsg);
                aFrontiersMapPub.publish(aFrontiersMap);

                aSeq += 1;
            }
            
            aFrontiersClustersPub.publish(aFrontierMsg);

            aState = FrontierState::IDLE;
        break;
    }
}

/*
 * Node's main function
 */
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "frontierdiscoverynode");
    std::unique_ptr<FrontierDiscoveryNode> frontier_discovery = std::make_unique<FrontierDiscoveryNode>();
    ros::spin();
}