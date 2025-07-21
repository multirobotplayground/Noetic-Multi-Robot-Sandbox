/* 
 * Noetic-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
 * Copyright (C) 2023 Alysson Ribeiro da Silva
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

#include "IntegratedGlobalPlannerNode.h"

IntegratedGlobalPlannerNode::IntegratedGlobalPlannerNode() {
    ros::NodeHandle node_handle("~");

    // Load all parameters
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("reach_threshold", aSubGoalReachThreshold)) aSubGoalReachThreshold = 0.3;
    if(!node_handle.getParam("stuck_time_threshold", aStuckTimeThreshold)) aStuckTimeThreshold = 60.0;
    aNamespace = ros::this_node::getNamespace();

    aSeq = 0;
    aDistance = 0.0;
    aStuckTime = 0.0;
    aDeltaTimeSec = 0.0;
    aAverageVelocity = 0.0;
    aHasOcc = false;
    aHasPose = false;
    aHasAverageVelocity = false;

    // Subscriptions
    aSubscribers.push_back(
        node_handle.subscribe<nav_msgs::OccupancyGrid>(
            aNamespace + "/c_space_path_plan", 
            aQueueSize, 
            std::bind(&IntegratedGlobalPlannerNode::CSpaceCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::CustomPose>(
            aNamespace + "/gmapping_pose/world_pose", 
            aQueueSize, 
            std::bind(&IntegratedGlobalPlannerNode::PoseCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Float32>(
            aNamespace + "/average_velocity", 
            aQueueSize, 
            std::bind(&IntegratedGlobalPlannerNode::AverageVelocityCallback, this, std::placeholders::_1)));
    
    aSubscribers.push_back(
        node_handle.subscribe<geometry_msgs::Pose>(
            aNamespace + "/integrated_global_planner/goal", 
            aQueueSize, 
            std::bind(&IntegratedGlobalPlannerNode::GoalCallback, this, std::placeholders::_1)));
    
    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/integrated_global_planner/stop", 
            aQueueSize,
            std::bind(&IntegratedGlobalPlannerNode::StopCallBack, this, std::placeholders::_1)));

    // Advertisers
    aPathMarkerPublisher =  node_handle.advertise<visualization_msgs::Marker>(aNamespace + "/integrated_global_planner/path", aQueueSize);
    aFinishEventPublisher = node_handle.advertise<std_msgs::String>(aNamespace + "/integrated_global_planner/finish", aQueueSize);
    aCurrentPathPublisher = node_handle.advertise<nav_msgs::Path>(aNamespace + "/integrated_global_planner/current_path", aQueueSize);

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&IntegratedGlobalPlannerNode::Update, this)));
}

IntegratedGlobalPlannerNode::~IntegratedGlobalPlannerNode() {

}

void IntegratedGlobalPlannerNode::ChangeState(const SubGoalState& newState) {
    ROS_INFO("[IntegrateedGlobalPlanner] State change %d -> %d.", aCurrentState, newState);
    aCurrentState = newState;
}

void IntegratedGlobalPlannerNode::DepthFirstSearchFreePath(nav_msgs::OccupancyGrid& cspace, 
                                                            Vec2i& occpos,
                                                            Vec2i& target, 
                                                            Vec2i& closest,
                                                            std::list<Vec2i>& outpath) {
    // Clear output path
    outpath.clear();
    
    // Input validation
    if(!sa::IsInBounds(cspace, occpos)) {
        ROS_WARN("[IntegratedGlobalPlanner] occpos (%d,%d) is out of bounds", occpos.x, occpos.y);
        return;
    }
    
    // Check if start position is passable
    int occpos_idx = occpos.y * cspace.info.width + occpos.x;
    if(cspace.data[occpos_idx] > 50) {
        ROS_WARN("[IntegratedGlobalPlanner] occpos (%d,%d) is not passable", occpos.x, occpos.y);
        return;
    }

    // Create a copy of target and clamp to bounds
    Vec2i target_copy = target;
    if(target_copy.x >= static_cast<int>(cspace.info.width)) target_copy.x = cspace.info.width - 1;
    if(target_copy.y >= static_cast<int>(cspace.info.height)) target_copy.y = cspace.info.height - 1;
    if(target_copy.x < 0) target_copy.x = 0;
    if(target_copy.y < 0) target_copy.y = 0;

    // Try direct path to target first
    sa::ComputePath(cspace, occpos, target_copy, outpath);
    if(!outpath.empty()) {
        closest = target_copy;
        ROS_DEBUG("[IntegratedGlobalPlanner] Direct path to target found");
        return;
    }

    // Search in expanding circles around target to find nearest reachable point
    double best_distance_to_target = std::numeric_limits<double>::infinity();
    Vec2i best_reachable_point = occpos;
    bool found_reachable = false;
    
    // Maximum search radius to prevent infinite search
    int max_radius = std::min({50, static_cast<int>(cspace.info.width/2), static_cast<int>(cspace.info.height/2)});
    
    for(int radius = 1; radius <= max_radius; ++radius) {
        // Check all points at this radius from target
        for(int dx = -radius; dx <= radius; ++dx) {
            for(int dy = -radius; dy <= radius; ++dy) {
                // Only check points on the circle boundary (Manhattan distance = radius)
                if(std::abs(dx) + std::abs(dy) != radius) continue;
                
                Vec2i candidate = Vec2i::Create(target_copy.x + dx, target_copy.y + dy);
                
                // Check if candidate is within bounds and in free space
                if(sa::IsInBounds(cspace, candidate)) {
                    int candidate_idx = candidate.y * cspace.info.width + candidate.x;
                    
                    if(cspace.data[candidate_idx] <= 50) {
                        // Try to find path to this candidate
                        std::list<Vec2i> temp_path;
                        sa::ComputePath(cspace, occpos, candidate, temp_path);
                        
                        if(!temp_path.empty()) {
                            // Calculate distance from candidate to original target
                            double dist_to_target = sqrt(pow(candidate.x - target_copy.x, 2) + 
                                                        pow(candidate.y - target_copy.y, 2));
                            
                            // Keep track of the closest reachable point to target
                            if(dist_to_target < best_distance_to_target) {
                                best_distance_to_target = dist_to_target;
                                best_reachable_point = candidate;
                                outpath = std::move(temp_path);
                                found_reachable = true;
                            }
                        }
                    }
                }
            }
        }
        
        // If we found any reachable point at this radius, we can stop
        // since we're searching in expanding circles, this is the nearest
        if(found_reachable) {
            closest = best_reachable_point;
            ROS_DEBUG("[IntegratedGlobalPlanner] Found nearest reachable point (%d,%d) at distance %.2f from target (%d,%d)", 
                     closest.x, closest.y, best_distance_to_target, target_copy.x, target_copy.y);
            return;
        }
    }
    
    // If no reachable point found, fallback to current position
    if(!found_reachable) {
        ROS_WARN("[IntegratedGlobalPlanner] No reachable point found within radius %d of target (%d,%d)", 
                 max_radius, target_copy.x, target_copy.y);
        closest = occpos;
        outpath.clear();
    }
}

void IntegratedGlobalPlannerNode::CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq) {
    input.id = id;
    input.header.frame_id = "robot_" + std::to_string(id) + std::string("/map");
    input.header.stamp = ros::Time().now();
    input.ns = ns;
    input.points.clear();
    input.type = visualization_msgs::Marker::LINE_STRIP;
    input.action = visualization_msgs::Marker::MODIFY;
    input.pose.position = aCspace.info.origin.position;
    input.pose.orientation.x = 0.0;
    input.pose.orientation.y = 0.0;
    input.pose.orientation.z = 0.0;
    input.pose.orientation.w = 1.0;
    input.scale.x = 0.25;
    input.scale.y = 0.25;
    input.scale.z = 0.5;
    input.color.a = 1.0;
    input.color.r = 1.0;
    input.color.g = 0.0;
    input.color.b = 0.0;
    input.lifetime = ros::Duration(1);
}

void IntegratedGlobalPlannerNode::CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    if(!aHasOcc) aHasOcc = true;
    aCspace.data.assign(msg->data.begin(), msg->data.end());
    aCspace.header = msg->header;
    aCspace.info = msg->info;
}

void IntegratedGlobalPlannerNode::PoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    if(!aHasPose) aHasPose = true;
    aWorldPos.setX(msg->pose.position.x);
    aWorldPos.setY(msg->pose.position.y);
    aWorldPos.setZ(0.0);
}

void IntegratedGlobalPlannerNode::AverageVelocityCallback(std_msgs::Float32::ConstPtr msg) {
    if(!aHasAverageVelocity) aHasAverageVelocity = true;
    aAverageVelocity = msg->data;
}

void IntegratedGlobalPlannerNode::GoalCallback(geometry_msgs::Pose::ConstPtr msg) {
    aCurrentGoal.setX(msg->position.x);
    aCurrentGoal.setY(msg->position.y);
    aCurrentGoal.setZ(0.0);
    ChangeState(state_executing_path);
}

void IntegratedGlobalPlannerNode::StopCallBack(std_msgs::String::ConstPtr msg) {
    aWaypoints.clear();
    ChangeState(state_idle);
}

void IntegratedGlobalPlannerNode::Update() {
    if(!aHasOcc || !aHasPose || !aHasAverageVelocity) return;
    
    WorldToMap(aCspace, aWorldPos, aOccPos);
    aPathMsg.poses.clear();

    // temp goal is utilized to help
    // checking if the current goal can be reached
    Vec2i temp_goal;
    
    switch(aCurrentState) {
        case state_idle:
            // do nothing
        break;
        case state_executing_path:
            // goal conversion to grid coordinates
            // to avoid wrong conversions due to 
            // the OCC dynamic nature
            WorldToMap(aCspace, aCurrentGoal, temp_goal);

            /*
             * Compute a path from the cell position to the selected free space
             * that is near to the frontier estimate pose
             */
            Vec2i closest_reachable_point;
            DepthFirstSearchFreePath(aCspace, 
                                        aOccPos, 
                                        temp_goal, 
                                        closest_reachable_point,
                                        aWaypoints);

            // check if it reached the goal - use the closest reachable point found
            tf::Vector3 closest_world_pos;
            MapToWorld(aCspace, closest_reachable_point, closest_world_pos);
            aDistance = aWorldPos.distance(closest_world_pos);

            if(aDistance <= aSubGoalReachThreshold) {
                aFinishEventPublisher.publish(aStrMsg);
                aWaypoints.clear();
                
                ROS_INFO("[IntegrateedGlobalPlanner] path ended.");      
                ChangeState(state_idle);                  
            } else {
                if(aWaypoints.size() > 0 ) {
                    CreateMarker(aPathMarkerMsg, aNamespace.c_str(), aId, aSeq);

                    for(auto& lit : aWaypoints) {
                        tf::Vector3 world;
                        MapToWorld(aCspace, lit, world);

                        // this adds a little filter on paths
                        // double lit_to_obj = world.distance(aCurrentGoal);
                        // double pose_to_obj = aWorldPos.distance(aCurrentGoal);
                        // if(pose_to_obj < lit_to_obj) continue;

                        geometry_msgs::Point pose;
                        pose.x = world.getX();
                        pose.y = world.getY();
                        pose.z = 0.0;
                        aPathMarkerMsg.points.push_back(pose);

                        geometry_msgs::PoseStamped pose_msg;
                        pose_msg.header = aCspace.header;
                        pose_msg.pose.orientation = geometry_msgs::Quaternion();

                        pose_msg.pose.position.x = world.getX();
                        pose_msg.pose.position.y = world.getY();

                        // publish individual waypoint
                        aPathMsg.poses.push_back(pose_msg);
                    }

                    // check if it is stuck
                    if(aAverageVelocity < 0.01) {
                        aDeltaTimeSec = ros::Time::now().sec - last_time.sec;
                        aStuckTime += aDeltaTimeSec;
                    } else {
                        aStuckTime = 0.0;
                    }
                } 

                if(aStuckTime > aStuckTimeThreshold) {
                    aFinishEventPublisher.publish(aStrMsg);

                    aWaypoints.clear();
                    aStuckTime = 0.0;

                    ROS_INFO("[IntegrateedGlobalPlanner] should reset planner.");   
                    ChangeState(state_idle);
                }
            }
            
        break;
    }

    // send markers and path
    aPathMarkerPublisher.publish(aPathMarkerMsg);
    aCurrentPathPublisher.publish(aPathMsg);

    last_time = ros::Time::now();
    aSeq++;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "integratedglobalplannernode");
    std::unique_ptr<IntegratedGlobalPlannerNode> subgoalHandlerNode = std::make_unique<IntegratedGlobalPlannerNode>();
    ros::spin();    
}