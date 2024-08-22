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
                                                            Vec2i& source, 
                                                            Vec2i& closest,
                                                            std::list<Vec2i>& outpath) {
    // used to mark found frontiers and clusters
    Vec2i source_copy = source;
    Matrix<bool> visited(cspace.info.width, cspace.info.height);
    visited.clear(0);

    // filter source to maximum cell decomposition bounds
    if(source_copy.x >= cspace.info.width) source_copy.x = cspace.info.width - 1;
    if(source_copy.y >= cspace.info.height) source_copy.y = cspace.info.height - 1;
    if(source_copy.x < 0) source_copy.x = 0;
    if(source_copy.y < 0) source_copy.y = 0;

    std::queue<Vec2i> q;
    q.push(source_copy);
    Vec2i current;
    visited[source_copy.y][source_copy.x] = true;
    while(q.size() > 0) {
        current = q.front();
        q.pop();
        sa::ComputePath(cspace, occpos, current, outpath);
        if(outpath.size() != 0) {
            closest = current;
            break;
        }

        for(int col = 0; col < 3; ++col) {
            for(int row = 0; row < 3; ++row) {
                Vec2i temp = Vec2i::Create(current.x - 1 + col,current.y - 1 + row);
                if(sa::IsInBounds(cspace, temp) && col != row && visited[temp.y][temp.x] == false) {
                    q.push(temp);
                    visited[temp.y][temp.x] = true;
                }
            }
        }
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
            DepthFirstSearchFreePath(aCspace, 
                                        aOccPos, 
                                        temp_goal, 
                                        temp_goal,
                                        aWaypoints);

            // check if it reached the goal
            MapToWorld(aCspace, temp_goal, aCurrentGoal);
            aDistance = aWorldPos.distance(aCurrentGoal);

            if(aDistance <= aSubGoalReachThreshold) {
                aFinishEventPublisher.publish(aStrMsg);
                aWaypoints.clear();
                
                ROS_INFO("[IntegrateedGlobalPlanner] path ended.");      
                ChangeState(state_idle);                  
            } else {
                if(aWaypoints.size() > 0 ) {
                    CreateMarker(aPathMarkerMsg, aNamespace.c_str(), aId, aSeq);

                    for(auto& lit : aWaypoints) {
                        geometry_msgs::Point pose;
                        pose.x = lit.x * aCspace.info.resolution;
                        pose.y = lit.y * aCspace.info.resolution;
                        pose.z = 0.0;
                        aPathMarkerMsg.points.push_back(pose);

                        geometry_msgs::PoseStamped pose_msg;
                        pose_msg.header = aCspace.header;
                        pose_msg.pose.orientation = geometry_msgs::Quaternion();

                        tf::Vector3 world;
                        MapToWorld(aCspace, lit, world);
                        
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

                if(aStuckTime > aStuckTimeThreshold || aWaypoints.size() == 0) {
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