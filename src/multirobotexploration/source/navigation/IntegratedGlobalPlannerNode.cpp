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
#include "SearchAlgorithms.h"

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
            aNamespace + "/c_space", 
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

    // Simple BFS from target to find any reachable point
    Matrix<bool> visited(cspace.info.height, cspace.info.width);
    visited.clear(false);
    std::queue<Vec2i> q;
    q.push(target_copy);
    visited[target_copy.y][target_copy.x] = true;
    
    // Add iteration limit to prevent infinite loops
    int max_iterations = 500;
    int iterations = 0;
    
    while(!q.empty() && iterations < max_iterations) {
        iterations++;
        Vec2i current = q.front();
        q.pop();

        // Test if current point is reachable from robot
        sa::ComputePath(cspace, occpos, current, outpath);
        if(!outpath.empty()) {
            closest = current;
            ROS_DEBUG("[IntegratedGlobalPlanner] Found reachable point (%d,%d) from target (%d,%d) after %d iterations", 
                      closest.x, closest.y, target_copy.x, target_copy.y, iterations);
            return;
        }

        // Add neighbors to queue
        for(int dx = -1; dx <= 1; dx++) {
            for(int dy = -1; dy <= 1; dy++) {
                if(dx == 0 && dy == 0) continue; // Skip current position
                
                Vec2i neighbor = Vec2i::Create(current.x + dx, current.y + dy);
                
                if(sa::IsInBounds(cspace, neighbor) && !visited[neighbor.y][neighbor.x]) {
                    int neighbor_idx = neighbor.y * cspace.info.width + neighbor.x;
                    visited[neighbor.y][neighbor.x] = true;
                    q.push(neighbor);
                }
            }
        }
    }
    
    // If exceeded max iterations, warn about potential infinite loop
    if(iterations >= max_iterations) {
        ROS_WARN("[IntegratedGlobalPlanner] BFS search exceeded maximum iterations (%d), potential infinite loop prevented", max_iterations);
    }
    
    // If no reachable point found, fallback to current position
    ROS_WARN("[IntegratedGlobalPlanner] No reachable point found from target (%d,%d) after %d iterations", 
             target_copy.x, target_copy.y, iterations);
    closest = occpos;
    outpath.clear();
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
    
    // Initialize last_time on first call
    static bool first_call = true;
    if(first_call) {
        last_time = ros::Time::now();
        first_call = false;
    }

    WorldToMap(aCspace, aWorldPos, aOccPos);
    aPathMsg.poses.clear();

    // Validate occupancy grid conversion
    if(aOccPos.x < 0 || aOccPos.y < 0 || 
       aOccPos.x >= static_cast<int>(aCspace.info.width) || 
       aOccPos.y >= static_cast<int>(aCspace.info.height)) {
        ROS_ERROR("[IntegratedGlobalPlanner] Robot position converts to invalid grid coordinates: (%d,%d). World pos: (%.3f, %.3f)", 
                  aOccPos.x, aOccPos.y, aWorldPos.getX(), aWorldPos.getY());
        return;
    }

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

            // Validate goal conversion
            if(temp_goal.x < 0 || temp_goal.y < 0 || 
               temp_goal.x >= static_cast<int>(aCspace.info.width) || 
               temp_goal.y >= static_cast<int>(aCspace.info.height)) {
                ROS_ERROR("[IntegratedGlobalPlanner] Goal converts to invalid grid coordinates: (%d,%d). World goal: (%.3f, %.3f)", 
                          temp_goal.x, temp_goal.y, aCurrentGoal.getX(), aCurrentGoal.getY());
                ChangeState(state_idle);
                break;
            }
            ROS_INFO("[IntegratedGlobalPlanner] World Pos: (%.3f, %.3f). World goal: (%.3f, %.3f)", 
                        aWorldPos.getX(), aWorldPos.getY(), aCurrentGoal.getX(), aCurrentGoal.getY());

            /*
             * Always recompute path to handle dynamic environments and robot movement
             * This prevents the robot from getting stuck with outdated paths
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
                
                ROS_INFO("[IntegratedGlobalPlanner] Goal reached, distance: %.3f", aDistance);      
                ChangeState(state_idle);                  
            } else {
                // Process waypoints and filter out those behind the robot
                CreateMarker(aPathMarkerMsg, aNamespace.c_str(), aId, aSeq);
                
                bool has_valid_waypoints = false;
                
                for(auto& lit : aWaypoints) {
                    tf::Vector3 world;
                    MapToWorld(aCspace, lit, world);

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

                    aPathMsg.poses.push_back(pose_msg);
                }

                if(aAverageVelocity < 0.01) {
                    ros::Time current_time = ros::Time::now();
                    if(current_time.sec >= last_time.sec) { // Prevent negative time differences
                        aDeltaTimeSec = current_time.sec - last_time.sec;
                        aStuckTime += aDeltaTimeSec;
                    } else {
                        ROS_WARN("[IntegratedGlobalPlanner] Time moved backwards during stuck detection, resetting timer");
                        aStuckTime = 0.0;
                    }

                } else {
                    aStuckTime = 0.0;
                }

                // Reset planner if stuck too long
                if(aStuckTime > aStuckTimeThreshold) {
                    aFinishEventPublisher.publish(aStrMsg);
                    aWaypoints.clear();
                    aStuckTime = 0.0;
                    ROS_INFO("[IntegratedGlobalPlanner] Robot stuck for too long, resetting planner");   
                    ChangeState(state_idle);
                }
            }
            
        break;
    }

    // Always send markers and path (even if empty to clear old data)
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