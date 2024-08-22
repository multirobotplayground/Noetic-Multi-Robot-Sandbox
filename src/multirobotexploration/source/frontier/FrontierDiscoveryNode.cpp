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

#include "FrontierDiscoveryNode.h"

/*
 * Node implementation
 */
FrontierDiscoveryNode::FrontierDiscoveryNode() {
    aState = FrontierState::IDLE;

    // load all parameters
    ros::NodeHandle node_handle("~");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve robot id.");
    if(!node_handle.getParam("max_lidar_range", aMaxLidarRange)) aMaxLidarRange = 10.0;
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
            aNamespace + "/gmapping_pose/world_pose", 
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

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&FrontierDiscoveryNode::Update, this)));
}

FrontierDiscoveryNode::~FrontierDiscoveryNode() {
    
}

void FrontierDiscoveryNode::CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    aOcc.data.assign(msg->data.begin(), msg->data.end());
    aOcc.info = msg->info;
    aOcc.header = msg->header;
    aReceivedCSpace = true;
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
    input.scale.x = 0.25;
    input.scale.y = 0.25;
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
    double range_squared = lidarRange * lidarRange;
    int range_in_cells = static_cast<int>(lidarRange / occ.info.resolution);

    Vec2i min = Vec2i::Create(centroid.x-range_in_cells,centroid.y-range_in_cells);
    Vec2i max = Vec2i::Create(centroid.x+range_in_cells,centroid.y+range_in_cells);

    // clamp the ranges
    min.x = std::max(0, min.x);
    min.y = std::max(0, min.y);
    max.x = std::min(max.x, static_cast<int>(occ.info.width));
    max.y = std::min(max.y, static_cast<int>(occ.info.height));

    // count area in cells
    int count = 0;
    for(int x = min.x; x < max.x; ++x) {
        for(int y = min.y; y < max.y; ++y) {
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
                int index = y * occ.info.width + x;
                if(occ.data[index] == -1) {
                    count++;
                }
                
            }
        }
    }

    double cell_area = occ.info.resolution * occ.info.resolution;
    double total_area_value = static_cast<double>(count) * cell_area;
    ROS_INFO("[FrontierDiscovery] Cells: %d area to uncover in meters squared: %f", count, total_area_value);
    return total_area_value;
}

void FrontierDiscoveryNode::Update() {
    if(aReceivedCSpace == false || aHasPose == false) return;
    
    double cost, value, utility;

    switch(aState) {
        case IDLE:
            // do nothing
        break;
        case PROCESSING:
            WorldToMap(aOcc, aWorldPos, aPos);

            // compute frontiers from c_space
            sa::ComputeFrontiers(aOcc, aFrontiersMap, aFrontiers);
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
                sa::ComputePath(aOcc, aPos, aCentroids[i], aPath);

                // compute convex hull of the frontiers
                if(aPath.size() > 0) {
                    cost = static_cast<double>(aPath.size()) * static_cast<double>(aOcc.info.resolution);
                    value = ComputeCentroidValue(aOcc, aCentroids[i], aMaxLidarRange);
                    utility = value / cost;

                    aFilteredCentroids.push_back(aCentroids[i]);
                    aFrontierMsg.costs.data.push_back(cost);
                    aFrontierMsg.values.data.push_back(value);
                    aFrontierMsg.utilities.data.push_back(utility);
                }
            }

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