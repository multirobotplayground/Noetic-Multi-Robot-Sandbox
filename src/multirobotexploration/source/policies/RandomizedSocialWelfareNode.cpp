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

#include "RandomizedSocialWelfareNode.h"

RandomizedSocialWelfareNode::RandomizedSocialWelfareNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots.");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    aNamespace = ros::this_node::getNamespace();

    aHasOcc = false;
    aHasPose = false;
    aHasComm = false;
    aFirst = true;
    aDirty = true;
    aCurrentState = state_idle;

    // initialize containers
    aRandomNumberGenerator = std::make_unique<std::mt19937>(aRandomNumberDevice());

    // Subscriptions
    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::Frontiers>(
            aNamespace + "/frontier_discovery/frontiers_clusters", 
            aQueueSize, std::bind(&RandomizedSocialWelfareNode::ClustersCallback, this, std::placeholders::_1)));
    
    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::CustomPose>(
            aNamespace + "/gmapping_pose/world_pose", 
            aQueueSize, 
            std::bind(&RandomizedSocialWelfareNode::EstimatePoseCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/integrated_global_planner/finish", 
            aQueueSize, 
            std::bind(&RandomizedSocialWelfareNode::SubGoalFinishCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<nav_msgs::OccupancyGrid>(
            aNamespace + "/c_space", 
            aQueueSize, 
            std::bind(&RandomizedSocialWelfareNode::CSpaceCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/explorer/set_idle", 
            aQueueSize, 
            std::bind(&RandomizedSocialWelfareNode::SetIdleCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/explorer/set_exploring", 
            aQueueSize, 
            std::bind(&RandomizedSocialWelfareNode::SetExploringCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            "/global_explorer/back_to_base", 
            aQueueSize, 
            std::bind(&RandomizedSocialWelfareNode::SetBasestationCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            "/global_explorer/set_exploring", 
            aQueueSize, 
            std::bind(&RandomizedSocialWelfareNode::SetExploringCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Int8MultiArray>(
            aNamespace + "/mock_communication_model/robots_in_comm", 
            aQueueSize,
            std::bind(&RandomizedSocialWelfareNode::CommCallback, this, std::placeholders::_1)));

    // Advertisers
    aGoalPublisher = node_handle.advertise<geometry_msgs::Pose>(aNamespace + "/integrated_global_planner/goal", aQueueSize);
    aFrontierComputePublisher = node_handle.advertise<std_msgs::String>(aNamespace + "/frontier_discovery/compute", aQueueSize);

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&RandomizedSocialWelfareNode::Update, this)));
}

RandomizedSocialWelfareNode::~RandomizedSocialWelfareNode() {

}

void RandomizedSocialWelfareNode::ClustersCallback(multirobotsimulations::Frontiers::ConstPtr msg) {
    aFrontierCentroidsMsg.centroids.poses.assign(msg->centroids.poses.begin(), msg->centroids.poses.end());
    aFrontierCentroidsMsg.centroids.header = msg->centroids.header;
    aFrontierCentroidsMsg.costs.data.assign(msg->costs.data.begin(), msg->costs.data.end());
    aFrontierCentroidsMsg.utilities.data.assign(msg->utilities.data.begin(), msg->utilities.data.end());

    aFrontierCentroidsMsg.highest_cost_index = msg->highest_cost_index;
    aFrontierCentroidsMsg.highest_value_index = msg->highest_value_index;
    aFrontierCentroidsMsg.highest_utility_index = msg->highest_utility_index;

    aFrontierCentroidsMsg.highest_cost = msg->highest_cost;
    aFrontierCentroidsMsg.highest_value = msg->highest_value;
    aFrontierCentroidsMsg.highest_utility = msg->highest_utility;

    ChangeState(state_select_frontier);
}

void RandomizedSocialWelfareNode::EstimatePoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    if(!aHasPose) aHasPose = true;
    aWorldPos.setX(msg->pose.position.x);
    aWorldPos.setY(msg->pose.position.y);
}

void RandomizedSocialWelfareNode::SubGoalFinishCallback(std_msgs::String::ConstPtr msg) {
    if(aCurrentState == state_exploring) ChangeState(state_exploration_finished);
    if(aCurrentState == state_back_to_base) ChangeState(state_back_to_base_finished);
}

void RandomizedSocialWelfareNode::CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    if(!aHasOcc) aHasOcc = true;
    aCSpaceMsg.info = msg->info;
    aCSpaceMsg.header = msg->header;
}

void RandomizedSocialWelfareNode::SetIdleCallback(std_msgs::String::ConstPtr msg) {
    ChangeState(state_idle);
}

void RandomizedSocialWelfareNode::SetBasestationCallback(std_msgs::String::ConstPtr msg) {
    ChangeState(state_set_back_to_base);
}

void RandomizedSocialWelfareNode::SetExploringCallback(std_msgs::String::ConstPtr msg) { 
    ChangeState(state_compute_centroids);
}

int RandomizedSocialWelfareNode::SelectFrontier(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld) {
    selectFrontierWorld.setX(centroids.centroids.poses[centroids.highest_utility_index].position.x);
    selectFrontierWorld.setY(centroids.centroids.poses[centroids.highest_utility_index].position.y);
    return centroids.highest_utility_index;
}

void RandomizedSocialWelfareNode::CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq) {
    input.id = id;
    input.header.frame_id = std::string("robot_") + std::to_string(id) + std::string("/map");
    input.header.stamp = ros::Time().now();
    input.ns = ns;
    input.points.clear();
    input.type = visualization_msgs::Marker::CYLINDER;
    input.action = visualization_msgs::Marker::MODIFY;
    input.pose.orientation.x = 0.0;
    input.pose.orientation.y = 0.0;
    input.pose.orientation.z = 0.0;
    input.pose.orientation.w = 1.0;
    input.scale.x = 0.5;
    input.scale.y = 0.5;
    input.scale.z = 0.5;
    input.color.a = 1.0;
    input.color.r = 0.3;
    input.color.g = 1.0;
    input.color.b = 0.0;
    input.lifetime = ros::Duration(1);
}

void RandomizedSocialWelfareNode::SetGoal(const tf::Vector3& goal) {
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = goal.getX();
    pose_msg.position.y = goal.getY();
    aGoalPublisher.publish(pose_msg);    
}

void RandomizedSocialWelfareNode::ChangeState(const ExplorerState& newState) {
    ROS_INFO("[RandomizedSocialWelfareNode] State change %d -> %d.", aCurrentState, newState);
    aCurrentState = newState;
}

bool RandomizedSocialWelfareNode::CheckNear() {
    for(size_t robot = 0; robot < aCommMsg.data.size(); ++robot) {
        if(robot == aId) continue;
        if(aCommMsg.data[robot] == 1) return true;
    }
    return false;
}

int RandomizedSocialWelfareNode::RandomizedFrontierSelection(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld) {
    // simply creates a distribution and samples a position from the centroids array
    std::uniform_int_distribution<int> distribution(0, centroids.centroids.poses.size()-1);
    int selected = distribution(*aRandomNumberGenerator); 
    selectFrontierWorld.setX(centroids.centroids.poses[selected].position.x);
    selectFrontierWorld.setY(centroids.centroids.poses[selected].position.y);
    return selected;  
}

void RandomizedSocialWelfareNode::CommCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    if(!aHasComm) aHasComm = true;
    aCommMsg.data.assign(msg->data.begin(), msg->data.end());
    aCommMsg.layout = msg->layout;
}

void RandomizedSocialWelfareNode::Update() {
    if(!aHasPose || !aHasOcc || !aHasComm) return;

    WorldToMap(aCSpaceMsg, aWorldPos, aOccPos);

    if(aDirty) {
        aGoalBasestation.setX(aWorldPos.getX());
        aGoalBasestation.setY(aWorldPos.getY());
        aDirty = false;
    }

    switch(aCurrentState) {
        case state_idle:
            // just wait for command
        break;

        case state_compute_centroids:
            // ask for centroids to avoid
            // unnecessary computations
            aFrontierComputePublisher.publish(std_msgs::String());
            ChangeState(state_waiting_centroids);
        break;

        case state_waiting_centroids:
            // just wait for the centroids to arrive
        break;

        case state_select_frontier:
            if(aFrontierCentroidsMsg.centroids.poses.size() == 0) {
                SetGoal(aGoalBasestation);
                ChangeState(state_set_back_to_base);
                ROS_INFO("[RandomizedSocialWelfareNode] Not more clusters to explore [%.2f %.2f]", 
                            aGoalBasestation.getX(), 
                            aGoalBasestation.getY());
                break;
            }
            
            if(aFrontierCentroidsMsg.centroids.poses.size() > 0) {
               if(CheckNear()) {
                    RandomizedFrontierSelection(aFrontierCentroidsMsg, aGoalFrontier);
                    ROS_INFO("[RandomizedSocialWelfareNode] randomized utility.");
                } else {
                    SelectFrontier(aFrontierCentroidsMsg, aGoalFrontier);
                    ROS_INFO("[RandomizedSocialWelfareNode] maximizing utility.");
                }
                ROS_INFO("[RandomizedSocialWelfareNode] selected frontier [%.2f %.2f]", 
                            aGoalFrontier.getX(),
                            aGoalFrontier.getY());

                SetGoal(aGoalFrontier);
                ChangeState(state_exploring);
            } else {
                ChangeState(state_set_back_to_base);
            }
        break;

        case state_set_back_to_base:
            SetGoal(aGoalBasestation);
            ROS_INFO("[RandomizedSocialWelfareNode] going back to base at [%.2f %.2f]", 
                        aGoalBasestation.getX(), 
                        aGoalBasestation.getY());   
            ChangeState(state_back_to_base);             
        break;

        case state_back_to_base:

        break;

        case state_back_to_base_finished:
            ROS_INFO("[RandomizedSocialWelfareNode] reached motherbase.");
            ChangeState(state_idle);
        break;

        case state_exploration_finished:
            ROS_INFO("[RandomizedSocialWelfareNode] reached frontier.");
            ChangeState(state_compute_centroids);
        break;
    }

    aDeltaTime = ros::Time::now().sec - aLastTime.sec;
    ROS_INFO("[RandomizedSocialWelfareNode] current state %d delta time: %f", aCurrentState, aDeltaTime);

    // run spin to get the data
    aLastTime = ros::Time::now();

    if(aFirst) aFirst = false;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "randomizedsocialwelfarenode");
    std::unique_ptr<RandomizedSocialWelfareNode> randomizedSocialWelfareNode = std::make_unique<RandomizedSocialWelfareNode>();
    ros::spin();
}