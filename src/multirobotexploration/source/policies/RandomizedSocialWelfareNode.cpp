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
            aNamespace + "/world_pose", 
            aQueueSize, 
            std::bind(&RandomizedSocialWelfareNode::EstimatePoseCallback, this, std::placeholders::_1)));

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

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Int32>(
            aNamespace + "/mock_communication_model/event", 
            aQueueSize, 
            std::bind(&RandomizedSocialWelfareNode::CommEvent, this, std::placeholders::_1)));

    // Initialize move_base action client
    aMoveBaseClient = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(
        aNamespace + "/move_base", true);

    // Wait for the action server to come up
    ROS_INFO("[RandomizedSocialWelfareNode] Waiting for move_base action server...");
    aMoveBaseClient->waitForServer(ros::Duration(30.0));

    if (!aMoveBaseClient->isServerConnected()) {
        ROS_ERROR("[RandomizedSocialWelfareNode] move_base action server not available!");
    } else {
        ROS_INFO("[RandomizedSocialWelfareNode] Connected to move_base action server");
    }

    // Advertisers
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
    aCSpaceMsg.data.assign(msg->data.begin(), msg->data.end());
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
    move_base_msgs::MoveBaseGoal move_base_goal;
    
    // Set the target pose
    move_base_goal.target_pose.header.frame_id = "robot_" + std::to_string(aId) + "/map";
    move_base_goal.target_pose.header.stamp = ros::Time::now();
    
    move_base_goal.target_pose.pose.position.x = goal.getX();
    move_base_goal.target_pose.pose.position.y = goal.getY();
    move_base_goal.target_pose.pose.position.z = 0.0;
    
    // Set orientation (facing forward)
    move_base_goal.target_pose.pose.orientation.x = 0.0;
    move_base_goal.target_pose.pose.orientation.y = 0.0;
    move_base_goal.target_pose.pose.orientation.z = 0.0;
    move_base_goal.target_pose.pose.orientation.w = 1.0;
    
    aMoveBaseClient->sendGoal(move_base_goal,
        std::bind(&RandomizedSocialWelfareNode::DoneCallback, this, std::placeholders::_1, std::placeholders::_2));
    
    ROS_INFO("[RandomizedSocialWelfareNode] Sent move_base goal: [%.2f, %.2f]", 
             goal.getX(), goal.getY());
}

void RandomizedSocialWelfareNode::DoneCallback(const actionlib::SimpleClientGoalState& state,
                                              const move_base_msgs::MoveBaseResultConstPtr& result) {
    if(aCurrentState == state_exploring) ChangeState(state_exploration_finished);
    if(aCurrentState == state_back_to_base) ChangeState(state_back_to_base_finished);
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
    // Update current communication state
    if(!aHasComm) aHasComm = true;
    aCommMsg.data.assign(msg->data.begin(), msg->data.end());
    aCommMsg.layout = msg->layout;
}

void RandomizedSocialWelfareNode::CommEvent(std_msgs::Int32::ConstPtr msg) {
    // Update current communication event
    if(aCurrentState == state_exploring) {
        SetGoal(aWorldPos);
        ChangeState(state_compute_centroids);
    }
}

void RandomizedSocialWelfareNode::Update() {
    if(!aHasPose || !aHasOcc || !aHasComm) return;

    WorldToMap(aCSpaceMsg, aWorldPos, aOccPos);

    if(aDirty) {
        aGoalBasestation.setX(aWorldPos.getX());
        aGoalBasestation.setY(aWorldPos.getY());
        aDirty = false;
    }

    int index, val;
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
                WorldToMap(aCSpaceMsg, aGoalFrontier, aFrontierOcc);
                SetGoal(aGoalFrontier);
                ChangeState(state_exploring);
            } else {
                ChangeState(state_set_back_to_base);
            }
        break;
        case state_exploring:
            index = aFrontierOcc.y * aCSpaceMsg.info.width + aFrontierOcc.x;
            if(index >= 0 && index < aCSpaceMsg.data.size()) {
                val = aCSpaceMsg.data[index];
                if(val > 50) {
                    ROS_INFO("[RandomizedSocialWelfareNode] frontier blocked, selecing another place to visit.");
                    SetGoal(aWorldPos);
                }
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

            // try to find frontiers one last time to ensure 
            // a bug didnt happened during exploration
            ChangeState(state_compute_centroids);
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