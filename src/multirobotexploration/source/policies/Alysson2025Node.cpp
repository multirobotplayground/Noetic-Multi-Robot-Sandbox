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

#include "Alysson2025Node.h"
#include "SearchAlgorithms.h"
#include <algorithm>
#include <numeric>
#include <vector>



Alysson2025Node::Alysson2025Node() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots.");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 1000;
    if(!node_handle.getParam("waiting_threshold", aWaitingThreshold)) aWaitingThreshold = 500.0;

    aNamespace = ros::this_node::getNamespace();

    aHasOcc = false;
    aHasPose = false;
    aHasComm = false;
    aFirst = true;
    aDirty = true;
    aTimeWaiting = 0.0;
    aReceivedNewRendezvousLocation = false; // Initialize flag
    aLastTime = ros::Time::now(); // Initialize time tracking
    aRendezvousMsg.robot_id = aId;
    aRendezvousNewPoseMsg.robot_id = aId;
    aCurrentState = state_idle;

    // initialize containers
    aRandomNumberGenerator = std::make_unique<std::mt19937>(aRandomNumberDevice());
    aPlan = std::make_shared<RendezvousPlan>(node_handle, aId);

    // initialize rendezvous plan
    aPlan->PrintLocations();
    aPlan->Print();

    // Subscriptions
    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::Frontiers>(
            aNamespace + "/frontier_discovery/frontiers_clusters", 
            aQueueSize, std::bind(&Alysson2025Node::ClustersCallback, this, std::placeholders::_1)));
    
    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::CustomPose>(
            aNamespace + "/world_pose", 
            aQueueSize, 
            std::bind(&Alysson2025Node::EstimatePoseCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<nav_msgs::OccupancyGrid>(
            aNamespace + "/c_space", 
            aQueueSize, 
            std::bind(&Alysson2025Node::CSpaceCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/explorer/set_idle", 
            aQueueSize, 
            std::bind(&Alysson2025Node::SetIdleCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/explorer/set_exploring", 
            aQueueSize, 
            std::bind(&Alysson2025Node::SetExploringCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            "/global_explorer/back_to_base", 
            aQueueSize, 
            std::bind(&Alysson2025Node::SetBasestationCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            "/global_explorer/set_exploring", 
            aQueueSize, 
            std::bind(&Alysson2025Node::SetExploringCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Int8MultiArray>(
            aNamespace + "/mock_communication_model/robots_in_comm", 
            aQueueSize,
            std::bind(&Alysson2025Node::CommCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Float32>(
            aNamespace + "/average_velocity", 
            aQueueSize, 
            std::bind(&Alysson2025Node::AverageVelocityCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Int32>(
            aNamespace + "/mock_communication_model/event", 
            aQueueSize, 
            std::bind(&Alysson2025Node::CommEvent, this, std::placeholders::_1)));

    for(int robot = 0; robot < aRobots; ++robot) {
        if(robot == aId) continue;

        bool* receivedRendezvousLocationPtr   = &aReceivedNewRendezvousLocation; 
        tf::Vector3* newRendezvousLocationPtr = &aNewRendezvousLocation;
        RendezvousPlan* planPtr               = aPlan.get();
        std_msgs::Int8MultiArray* commPtr     = &aCommMsg;

        aSubscribers.push_back(
            node_handle.subscribe<multirobotsimulations::rendezvous>(
                "/robot_" + std::to_string(robot) + "/realizing_plan",
                aQueueSize,
                [planPtr, commPtr, this](multirobotsimulations::rendezvous::ConstPtr msg) {
                    if(!planPtr || commPtr->data.empty()) return;  // Safety check
                    if(msg->plan == planPtr->GetCurrentAgreementUniqueID() && CanCommunicate(msg->robot_id))
                        planPtr->RealizePlan(msg->robot_id);
                }
            ));
        
        aSubscribers.push_back(
            node_handle.subscribe<multirobotsimulations::CustomPose>(
                "/robot_" + std::to_string(robot) + "/plan_updater",
                aQueueSize,
                [planPtr, commPtr, receivedRendezvousLocationPtr, newRendezvousLocationPtr, this](multirobotsimulations::CustomPose::ConstPtr msg) {
                    if(!planPtr || commPtr->data.empty()) return;  // Safety check
                    // this should work because robots cannot rendezvous at two 
                    // simultaneous places
                    int waiting_id_from_consensus = planPtr->GetCurrentAgreementConsensusID();

                    // realize plan if it is the same as mine
                    // always check if can communicate to simulate networking
                    if(waiting_id_from_consensus == msg->robot_id && CanCommunicate(msg->robot_id)) {
                        if(!(*receivedRendezvousLocationPtr)) (*receivedRendezvousLocationPtr) = true;
                        newRendezvousLocationPtr->setX(msg->pose.position.x);
                        newRendezvousLocationPtr->setY(msg->pose.position.y);
                        newRendezvousLocationPtr->setZ(msg->pose.position.z);
                    }
                }
            ));

        /*
         * This only triggers if the current plan has 2 robots, which implies that 
         * the robot that receives this is going to receive from its consensus master
         *
         * */
        aSubscribers.push_back(
            node_handle.subscribe<multirobotsimulations::CustomPose>(
                "/robot_" + std::to_string(robot) + "/pairwise_dynamic_updater",
                aQueueSize,
                [planPtr, commPtr, this](multirobotsimulations::CustomPose::ConstPtr msg) {
                    if(!planPtr || commPtr->data.empty()) return;
                    int current_plan_id = (int)msg->pose.position.z;
                    tf::Vector3 dynamic_rendezvous_location(msg->pose.position.x, msg->pose.position.y, 0.0);
                    if(current_plan_id == planPtr->GetCurrentAgreementUniqueID()) {
                        planPtr->UpdateCurrentAgreementLocation(dynamic_rendezvous_location);
                        ROS_INFO("[Alysson2024Node] Received pairwise rendezvous location [%.2f %.2f] for plan %d",
                                dynamic_rendezvous_location.x(), dynamic_rendezvous_location.y(), 
                                planPtr->GetCurrentAgreementUniqueID());
                    }
                }
            )
        );
    }

    // Advertisers
    aGoalPublisher = node_handle.advertise<geometry_msgs::Pose>(aNamespace + "/integrated_global_planner/goal", aQueueSize);
    aFrontierComputePublisher = node_handle.advertise<std_msgs::String>(aNamespace + "/frontier_discovery/compute", aQueueSize);
    aPlanRealizationPublisher = node_handle.advertise<multirobotsimulations::rendezvous>(aNamespace + "/realizing_plan", aQueueSize);
    aPlanLocationPublisher = node_handle.advertise<multirobotsimulations::CustomPose>(aNamespace + "/plan_updater", aQueueSize);
    aWaitingTimePublisher = node_handle.advertise<std_msgs::Float64MultiArray>(aNamespace + "/waiting_time_at_rendezvous", aQueueSize);
    aDistanceToRendezvousPublisher = node_handle.advertise<std_msgs::Float64MultiArray>(aNamespace + "/mission_metrics", aQueueSize);
    aClusterMarkerPub = node_handle.advertise<visualization_msgs::Marker>("/rendezvous_locations", aQueueSize);
    aPairwiseDynamicUpdaterPublisher = node_handle.advertise<multirobotsimulations::CustomPose>(aNamespace + "/pairwise_dynamic_updater", aQueueSize);

    aHasVerageVelocity = false;
    aAverageVelocity = 0.0;
    aTotalTime = 0.0;
    aTimeToReachNextRendezvous = aPlan->GetCurrentAgreementTimer();
    path_length_meters = 0.0;
    expected_speed = 0.4;
    expected_heuristic_error = 3.0;
    time_to_reach_next_rendezvous = 0.0;
    time_diff = 0.0;
    heuristic = 0.0;
    fulfill_lasting_rendezvous = false;
    aStartingTime = ros::Time::now().toSec();

    aMoveBaseClient = std::make_shared<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(
        aNamespace + "/move_base", true);   

    // Wait for the action server to come up
    ROS_INFO("[Alysson2025Node] Waiting for move_base action server...");
    aMoveBaseClient->waitForServer(ros::Duration(30.0));

    if (!aMoveBaseClient->isServerConnected()) {
        ROS_ERROR("[Alysson2025Node] move_base action server not available!");
    } else {
        ROS_INFO("[Alysson2025Node] Connected to move_base action server");
    }

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&Alysson2025Node::Update, this)));
}

Alysson2025Node::~Alysson2025Node() {

}


void Alysson2025Node::AverageVelocityCallback(std_msgs::Float32::ConstPtr msg) {
    aAverageVelocity = msg->data;
    aHasVerageVelocity = true;
}

void Alysson2025Node::ClustersCallback(multirobotsimulations::Frontiers::ConstPtr msg) {
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

    if(aCurrentState == state_waiting_centroids) ChangeState(state_select_frontier);
    // if(aCurrentState == state_exploring) {
    //     int selected_index = 0;
    //     int select_policy = -1;
    //     if(CheckNear())
    //         select_policy = aId;

    //     selected_index = SelectFrontier(aFrontierCentroidsMsg, aGoalFrontier, select_policy);
    //     if(selected_index >= 0) {
    //         SetGoal(aGoalFrontier);
    //     }
    // }
    if(aCurrentState == state_waiting_centroids_for_plan) ChangeState(state_select_new_rendezvous);
}

void Alysson2025Node::EstimatePoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    if(!aHasPose) aHasPose = true;
    aWorldPos.setX(msg->pose.position.x);
    aWorldPos.setY(msg->pose.position.y);
}

void Alysson2025Node::CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    if(!aHasOcc) aHasOcc = true;
    aCSpaceMsg.info = msg->info;
    aCSpaceMsg.header = msg->header;
    aCSpaceMsg.data = msg->data;
}

void Alysson2025Node::SetIdleCallback(std_msgs::String::ConstPtr msg) {
    ChangeState(state_idle);
}

void Alysson2025Node::SetBasestationCallback(std_msgs::String::ConstPtr msg) {
    ChangeState(state_set_back_to_base);
}

void Alysson2025Node::SetExploringCallback(std_msgs::String::ConstPtr msg) { 
    aStartingTime = ros::Time::now().toSec();
    ChangeState(state_compute_centroids);
}

int Alysson2025Node::SelectFrontier(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld, const int& id) {
    if(centroids.centroids.poses.empty()) {
        ROS_WARN("[Alysson2025Node] No frontier poses available in SelectFrontier");
        return -1;
    }
    
    // Check if utilities data is available and has the same size as poses
    if(centroids.utilities.data.empty() || centroids.utilities.data.size() != centroids.centroids.poses.size()) {
        ROS_WARN("[Alysson2025Node] Utilities data invalid or size mismatch, using first frontier");
        selectFrontierWorld.setX(centroids.centroids.poses[0].position.x);
        selectFrontierWorld.setY(centroids.centroids.poses[0].position.y);
        return 0;
    }
    
    // Create vector of indices and sort by utility (highest first)
    std::vector<size_t> indices(centroids.centroids.poses.size());
    std::iota(indices.begin(), indices.end(), 0);

    std::sort(indices.begin(), indices.end(), 
        [&centroids](size_t a, size_t b) {
            return centroids.utilities.data[a] > centroids.utilities.data[b];
        });
    
    // Select the frontier with highest utility (first in sorted order)
    size_t best_index;
    if(id >= 0 && id < indices.size())
        best_index = indices[id];
    else
        best_index = indices[0]; // Default to the first if id is out of bounds

    selectFrontierWorld.setX(centroids.centroids.poses[best_index].position.x);
    selectFrontierWorld.setY(centroids.centroids.poses[best_index].position.y);
    
    ROS_INFO("[Alysson2025Node] Selected frontier %zu with utility %.3f", 
             best_index, centroids.utilities.data[best_index]);
    
    return static_cast<int>(best_index);
}

int Alysson2025Node::SelectRendezvous(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld) {
    if(centroids.centroids.poses.empty()) {
        ROS_WARN("[Alysson2025Node] No frontier poses available in SelectFrontier");
        return -1;
    }
    
    // Check if utilities data is available and has the same size as poses
    if(centroids.utilities.data.empty() || centroids.utilities.data.size() != centroids.centroids.poses.size()) {
        ROS_WARN("[Alysson2025Node] Utilities data invalid or size mismatch, using first frontier");
        selectFrontierWorld.setX(centroids.centroids.poses[0].position.x);
        selectFrontierWorld.setY(centroids.centroids.poses[0].position.y);
        return 0;
    }
    
    // Create vector of indices and sort by utility (highest first)
    std::vector<size_t> indices(centroids.centroids.poses.size());
    std::iota(indices.begin(), indices.end(), 0);
    
    std::sort(indices.begin(), indices.end(), 
        [&centroids](size_t a, size_t b) {
            return centroids.costs.data[a] > centroids.costs.data[b];  // Changed to > for highest first
        });
    
    // Select the frontier with highest cost (first in sorted order)
    size_t best_index = indices[0];

    selectFrontierWorld.setX(centroids.centroids.poses[best_index].position.x);
    selectFrontierWorld.setY(centroids.centroids.poses[best_index].position.y);
    
    ROS_INFO("[Alysson2025Node] Selected frontier %zu with cost %.3f", 
             best_index, centroids.costs.data[best_index]);
    
    return static_cast<int>(best_index);
}

void Alysson2025Node::CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq) {
    input.id = id;
    input.header.frame_id = std::string("/map");
    input.header.stamp = ros::Time().now();
    input.ns = ns;
    input.type = visualization_msgs::Marker::SPHERE;
    input.action = visualization_msgs::Marker::MODIFY;
    input.pose.orientation.x = 0.0;
    input.pose.orientation.y = 0.0;
    input.pose.orientation.z = 0.0;
    input.pose.orientation.w = 1.0;
    input.scale.x = 1.0;
    input.scale.y = 1.0;
    input.scale.z = 1.0;
    input.color.a = 1.0;
    input.color.r = 1.0;
    input.color.g = 1.0;
    input.color.b = 0.0;
    input.lifetime = ros::Duration(1);
}

void Alysson2025Node::SetGoal(const tf::Vector3& goal) {
    move_base_msgs::MoveBaseGoal goal_msg;
    
    goal_msg.target_pose.header.frame_id = "robot_" + std::to_string(aId) + "/map";
    goal_msg.target_pose.header.stamp = ros::Time::now();
    goal_msg.target_pose.pose.position.x = goal.getX();
    goal_msg.target_pose.pose.position.y = goal.getY();
    goal_msg.target_pose.pose.position.z = 0.0;
    goal_msg.target_pose.pose.orientation.x = 0.0;
    goal_msg.target_pose.pose.orientation.y = 0.0;
    goal_msg.target_pose.pose.orientation.z = 0.0;
    goal_msg.target_pose.pose.orientation.w = 1.0;

    aMoveBaseClient->sendGoal(goal_msg,
        std::bind(&Alysson2025Node::DoneCallback, this, std::placeholders::_1, std::placeholders::_2));
}   

void Alysson2025Node::DoneCallback(const actionlib::SimpleClientGoalState& state,
                          const move_base_msgs::MoveBaseResultConstPtr& result) {
    if(aCurrentState == state_exploring) ChangeState(state_exploration_finished);
    else if(aCurrentState == state_back_to_base) ChangeState(state_back_to_base_finished);
    else if(aCurrentState == state_navigating_to_rendezvous) { 
        if(aWorldPos.distance(aGoalRendezvous) <= 5.0) {
            ChangeState(state_at_rendezvous);          
        } else {
            ChangeState(state_set_rendezvous_location);
        }
    }
}

void Alysson2025Node::CommEvent(std_msgs::Int32::ConstPtr msg) {
    if(aCurrentState == state_exploring) {
        ROS_INFO("[Alysson2025Node] Communication event received, stopping current exploration");
        SetGoal(aWorldPos);
        ChangeState(state_compute_centroids);
    }
}

std::string Alysson2025Node::GetStateName(const int& state) {
    switch(state) {
        case state_idle: return "idle";
        case state_exploring: return "exploring";
        case state_exploration_finished: return "exploration_finished";
        case state_compute_centroids: return "compute_centroids";
        case state_waiting_centroids: return "waiting_centroids";
        case state_select_frontier: return "select_frontier";
        case state_navigating_to_rendezvous: return "navigating_to_rendezvous";
        case state_at_rendezvous: return "at_rendezvous";
        case state_set_rendezvous_location: return "set_rendezvous_location";
        case state_set_back_to_base: return "set_back_to_base";
        case state_back_to_base: return "back_to_base";
        case state_back_to_base_finished: return "back_to_base_finished";
        case state_waiting_centroids_for_plan: return "waiting_centroids_for_plan";
        case state_select_new_rendezvous: return "select_new_rendezvous";
        case state_check_end_condition: return "check_end_condition";
        default: return "unknown_state";
    }
}

void Alysson2025Node::ChangeState(const ExplorerState& newState) {
    std::string current_state_name = GetStateName(aCurrentState);
    std::string new_state_name = GetStateName(newState);
    ROS_INFO("[Alysson2025Node] State change %s -> %s.", current_state_name.c_str(), new_state_name.c_str());
    aCurrentState = newState;
}

bool Alysson2025Node::CheckNear() {
    if(aCommMsg.data.empty()) {
        ROS_WARN("[Alysson2025Node] Communication data is empty in CheckNear");
        return false;
    }
    for(size_t robot = 0; robot < aCommMsg.data.size(); ++robot) {
        if((int)robot == aId) continue;
        if(aCommMsg.data[robot] == 1) return true;
    }
    return false;
}

bool Alysson2025Node::CanCommunicate(const int& id) {
    if(aCommMsg.data.empty()) {
        ROS_WARN("[Alysson2025Node] Communication data is empty in CanCommunicate");
        return false;
    }
    if(id < 0 || id >= (int)aCommMsg.data.size()) {
        ROS_WARN("[Alysson2025Node] Robot id %d is out of range [0, %zu) in CanCommunicate", id, aCommMsg.data.size());
        return false;  // Changed from throw to return false
    }
    if(aCommMsg.data[id] == 1) return true;
    return false;   
}

bool Alysson2025Node::FinishedMission() {
    return (aCurrentState == state_back_to_base ||
            aCurrentState == state_back_to_base_finished ||
            aCurrentState == state_set_back_to_base);
}

int Alysson2025Node::RandomizedFrontierSelection(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld) {
    // Check if there are any frontiers available
    if(centroids.centroids.poses.empty()) {
        ROS_WARN("[Alysson2025Node] No frontier poses available in RandomizedFrontierSelection");
        return -1;
    }
    // simply creates a distribution and samples a position from the centroids array
    std::uniform_int_distribution<int> distribution(0, centroids.centroids.poses.size()-1);
    int selected = distribution(*aRandomNumberGenerator); 
    selectFrontierWorld.setX(centroids.centroids.poses[selected].position.x);
    selectFrontierWorld.setY(centroids.centroids.poses[selected].position.y);
    return selected;  
}

int Alysson2025Node::SelectSubteamNewRendezvous(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld) {
    if(centroids.centroids.poses.empty()) {
        ROS_WARN("[Alysson2025Node] No frontier poses available in SelectSubteamNewRendezvous");
        return -1;
    }
    if(centroids.highest_cost_index < 0 || centroids.highest_cost_index >= centroids.centroids.poses.size()) {
        ROS_WARN("[Alysson2025Node] Invalid highest_cost_index %d, using first frontier", centroids.highest_cost_index);
        centroids.highest_cost_index = 0;
    }
    selectFrontierWorld.setX(centroids.centroids.poses[centroids.highest_cost_index].position.x);
    selectFrontierWorld.setY(centroids.centroids.poses[centroids.highest_cost_index].position.y);
    return centroids.highest_cost_index;
}

void Alysson2025Node::CommCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    if(!aHasComm) aHasComm = true;
    aCommMsg.data.assign(msg->data.begin(), msg->data.end());
    aCommMsg.layout = msg->layout;
}

void Alysson2025Node::Update() {
    if(!aHasPose || !aHasOcc || !aHasComm) return;

    WorldToMap(aCSpaceMsg, aWorldPos, aOccPos);

    aDeltaTime = (ros::Time::now() - aLastTime).toSec();
    ROS_DEBUG("[Alysson2025Node] current state %d delta time: %f", aCurrentState, aDeltaTime);

    if(aDirty) {
        aGoalBasestation.setX(aWorldPos.getX());
        aGoalBasestation.setY(aWorldPos.getY());
        aPlan->InitializeLocation(aWorldPos);
        aStartingTime = ros::Time::now().toSec();
        aDirty = false;
    }

    geometry_msgs::Point p;

    int index, val, selected_index;
    switch(aCurrentState) {
        case state_idle:
            // just wait for command
        break;

        case state_compute_centroids:
            // ask for centroids to avoid
            // unnecessary computations
            if(aPlan->HasValidAgreement()) {
                aFrontierComputePublisher.publish(std_msgs::String());
                ChangeState(state_waiting_centroids);
            } else {
                ChangeState(state_set_back_to_base);
            }
        break;

        case state_waiting_centroids:
            // just wait for the centroids to arrive
        break;

        case state_select_frontier:
            if(CheckNear()) {
                selected_index = SelectFrontier(aFrontierCentroidsMsg, aGoalFrontier, aId);
                ROS_INFO("[Alysson2025Node] randomized utility.");
            } else {
                selected_index = SelectFrontier(aFrontierCentroidsMsg, aGoalFrontier);
                ROS_INFO("[Alysson2025Node] maximizing utility.");
            }
            
            if(selected_index >= 0) {
                ROS_INFO("[Alysson2025Node] selected frontier [%.2f %.2f]", 
                            aGoalFrontier.getX(),
                            aGoalFrontier.getY());

                WorldToMap(aCSpaceMsg, aGoalFrontier, aFrontierOcc);
                SetGoal(aGoalFrontier);
                ChangeState(state_exploring);
            } else {
                ROS_WARN("[Alysson2025Node] Failed to select valid frontier, checking for rendezvous or going to base");
                if(aPlan->HasValidAgreement()) {
                    ChangeState(state_set_rendezvous_location); 
                } else {
                    ChangeState(state_set_back_to_base);
                }
            }
        break;

        case state_exploring:
            // aFrontierComputePublisher.publish(std_msgs::String());

            // Add bounds checking for frontier coordinates
            index = aFrontierOcc.y * aCSpaceMsg.info.width + aFrontierOcc.x;
            if(index >= 0 && index < aCSpaceMsg.data.size()) {
                val = aCSpaceMsg.data[index];
                #define OCCUPIED_THRESHOLD 50
                if(val > OCCUPIED_THRESHOLD) {
                    ROS_INFO("[Alysson2025Node] frontier blocked, selecting another place to visit.");
                    aMoveBaseClient->cancelGoal();
                    ChangeState(state_compute_centroids);
                }
            } else {
                ROS_WARN("[Alysson2025Node] Frontier index %d out of bounds [0, %zu), skipping check", index, aCSpaceMsg.data.size());
            }

            /*
             * Dinamically update rendezvous location with 
             * pairwise rule
             */
            // for(size_t robot = 0; robot < aCommMsg.data.size(); ++robot) {
            //     if((int)robot == aId) continue;
            //     if(aCommMsg.data[robot] == 1) {
            //         if(aPlan->PairwiseRule(robot) && aPlan->CheckConsensusCurrentPlan() == true) {
            //             selected_index = SelectFrontier(aFrontierCentroidsMsg, aGoalRendezvous);
            //             if(selected_index >= 0) {
            //                 ROS_INFO("[Alysson2024Node] Pairwise rendezvous location updated to [%.2f %.2f] for plan %d",
            //                     aGoalRendezvous.getX(), aGoalRendezvous.getY(),
            //                     aPlan->GetCurrentAgreementUniqueID());
            //                 aPlan->UpdateCurrentAgreementLocation(aGoalRendezvous);
            //                 aRendezvousNewPoseMsg.pose.position.x = aGoalRendezvous.getX();
            //                 aRendezvousNewPoseMsg.pose.position.y = aGoalRendezvous.getY();
            //                 aRendezvousNewPoseMsg.pose.position.z = (double)aPlan->GetCurrentAgreementUniqueID();
            //                 aPairwiseDynamicUpdaterPublisher.publish(aRendezvousNewPoseMsg);
            //             }
            //             break;
            //         }
            //     }
            // }
        break;

        case state_exploration_finished:
            ROS_INFO("[Alysson2025Node] reached frontier.");
            ChangeState(state_compute_centroids);
        break;
        
        case state_set_back_to_base:
            SetGoal(aGoalBasestation);
            ROS_INFO("[Alysson2025Node] going back to base at [%.2f %.2f]", 
                        aGoalBasestation.getX(), 
                        aGoalBasestation.getY());  
            
            // compute and send all statistics once the mission endend
            aStatsArr.Print();
            aStatsArr.Send(aWaitingTimePublisher);
            
            ChangeState(state_back_to_base);             
        break;

        case state_back_to_base:

        break;

        case state_back_to_base_finished:
            ROS_INFO("[Alysson2025Node] reached motherbase.");
            ChangeState(state_idle);
        break;

        case state_set_rendezvous_location:
            aGoalRendezvous = aPlan->GetCurrentAgreementLocation();
            ROS_INFO("[Alysson2025Node] fulfilling plan %d of unique id %d",
                aPlan->GetCurrentAgreement(), 
                aPlan->GetCurrentAgreementUniqueID());
            SetGoal(aGoalRendezvous);
            ChangeState(state_navigating_to_rendezvous);
        break;

        case state_navigating_to_rendezvous:
            // just wait until reaching the rendezvous footprint 
            // position
        break;

        case state_at_rendezvous:
            // broadcast the plan I'm trying to fulfill
            aRendezvousMsg.plan = aPlan->GetCurrentAgreementUniqueID();
            aPlanRealizationPublisher.publish(aRendezvousMsg);

            // this is a hack, check a better way to do it
            aPlan->RealizePlan(aId);

            // This robot is going to wait until it receives all callback
            // calls from others that can communicate and are trying to 
            // fulfill the same plan
            if(aPlan->WasPlanRealized() == true) {
                /*
                * Send statistics to the system only after the rendezvous was realized
                * to facilitate the analysis of waiting times
                */
                aStatsArr.Add(Stats(aId, aTimeWaiting, aPlan->GetCurrentAgreementUniqueID(), aTotalTime, aTimeToReachNextRendezvous));
                // log the waiting time
                
                ROS_INFO("[Alysson2025Node] !!! Published waiting time stats: robot=%d, time=%.2fs, plan=%d !!!", 
                        aId, aTimeWaiting, aPlan->GetCurrentAgreementUniqueID());
                
                if(aPlan->CheckConsensusCurrentPlan() == true) {
                    ChangeState(state_updating_plan);
                    ROS_INFO("[Alysson2025Node] I am consensus robot.");
                    aTimeWaiting = 0.0;
                } else {
                    ChangeState(state_waiting_consensus);
                    ROS_INFO("[Alysson2025Node] Waiting consensus.");
                    aTimeWaiting = 0.0;
                }
            } else {
                ROS_INFO("[Alysson2025Node] Waiting at rendezvous for %fs (threshold: %fs) for plan %d", 
                    aTimeWaiting, aWaitingThreshold, aPlan->GetCurrentAgreementUniqueID());

                if(aTimeWaiting > aWaitingThreshold) {
                    /*
                    * Send statistics even when timeout occurs to capture all waiting times
                    */
                    aStatsArr.Add(Stats(aId, aTimeWaiting, aPlan->GetCurrentAgreementUniqueID(), aTotalTime, aTimeToReachNextRendezvous));
                    
                    ROS_WARN("[Alysson2025Node] TIMEOUT: Waited %fs > %fs at rendezvous, publishing timeout stats", 
                            aTimeWaiting, aWaitingThreshold);
                    
                    aTimeWaiting = 0.0;
                    aPlan->SkipPlan();
                    ChangeState(state_compute_centroids);
                    ROS_INFO("[Alysson2025Node] No one went to rendezvous, reseting this plan.");
                }
            }

            aTimeWaiting += aDeltaTime;
        break;

        case state_updating_plan:
            // ask for centroids to avoid
            // unnecessary computations
            aFrontierComputePublisher.publish(aFrontierCentroidsMsg);
            ChangeState(state_waiting_centroids_for_plan);
            ROS_INFO("[Alysson2025Node] Consensus asking for frontiers to update rendezvous location.");
        break;

        case state_waiting_centroids_for_plan:
            // just wait
        break;

        /*
         * CONSENSUS DECISION!!!
         */
        case state_select_new_rendezvous:
            selected_index = SelectRendezvous(aFrontierCentroidsMsg, aGoalFrontier);
            
            if(selected_index >= 0) {
                aPlan->UpdatePlan(aGoalFrontier);
                aTimeToReachNextRendezvous += aPlan->GetCurrentAgreementTimer();

                ROS_INFO("[Alysson2025Node] New location selected [%f %f], sending to others.",
                    aGoalFrontier.getX(), aGoalFrontier.getY());

                aRendezvousNewPoseMsg.pose.position.x = aGoalFrontier.getX();
                aRendezvousNewPoseMsg.pose.position.y = aGoalFrontier.getY();
                aRendezvousNewPoseMsg.pose.position.z = 0.0;
            } else {
                aPlan->UpdatePlan(aGoalBasestation);
                ROS_INFO("[Alysson2025Node] New location selected [%f %f], sending to others.",
                    aGoalFrontier.getX(), aGoalFrontier.getY());

                aRendezvousNewPoseMsg.pose.position.x = aGoalBasestation.getX();
                aRendezvousNewPoseMsg.pose.position.y = aGoalBasestation.getY();
                aRendezvousNewPoseMsg.pose.position.z = 0.0;
            }

            ChangeState(state_compute_centroids);
            aPlanLocationPublisher.publish(aRendezvousNewPoseMsg);
        break;

        /*
         * SLAVE DECISION!!!
         */
        case state_waiting_consensus:                
            // waiting for new rendezvous location from the
            // consensus robot
            if(aReceivedNewRendezvousLocation) {
                ROS_INFO("[Alysson2025Node] received new rendezvous location [%f %f].",
                    aNewRendezvousLocation.getX(), 
                    aNewRendezvousLocation.getY());

                aPlan->UpdatePlan(aNewRendezvousLocation);
                aPlan->PrintCurrent();

                ChangeState(state_compute_centroids);

                aReceivedNewRendezvousLocation = false;
                aTimeWaiting = 0.0;
                aTimeToReachNextRendezvous += aPlan->GetCurrentAgreementTimer();
            } else {
                ROS_INFO("[Alysson2025Node] Waiting for new rendezvous location for %fs (threshold: %fs)", aTimeWaiting, aWaitingThreshold);

                if(aTimeWaiting > aWaitingThreshold) {
                    aStatsArr.Add(Stats(aId, aTimeWaiting, aPlan->GetCurrentAgreementUniqueID(), aTotalTime, aTimeToReachNextRendezvous));

                    ROS_WARN("[Alysson2025Node] CONSENSUS TIMEOUT: Waited %fs > %fs for consensus, publishing timeout stats", 
                             aTimeWaiting, aWaitingThreshold);
                    
                    aPlan->SkipPlan();
                    ChangeState(state_compute_centroids);
                    aTimeWaiting = 0.0;
                    aReceivedNewRendezvousLocation = false; // Reset flag on timeout
                    aTimeToReachNextRendezvous += aPlan->GetCurrentAgreementTimer();
                }
            }

            aTimeWaiting += aDeltaTime;
        break;
    }

    if(aCurrentState < state_set_rendezvous_location &&
        aCurrentState != state_idle &&
        FinishedMission() == false &&
        aPlan->GetCurrentAgreement() != -1) {  // Add safety check for valid plan
        path_length_meters = 0.0;
        expected_speed = 1.0;
        expected_heuristic_error = 1.0;
        time_to_reach_next_rendezvous = 0.0;
        time_diff = 0.0;
        heuristic = 0.0;

        // new policy, set current timer to 0.0 
        // in case the robot is going to take
        // to much time to reach the next location
        // this is a method to reduce waiting time
        // at rendezvous locations
        aGoalRendezvous = aPlan->GetCurrentAgreementLocation();
        Vec2i next_rendezvous_occ_pos;
        WorldToMap(aCSpaceMsg, aGoalRendezvous, next_rendezvous_occ_pos);
        std::list<Vec2i> path;
        sa::ComputePath(aCSpaceMsg, aOccPos, next_rendezvous_occ_pos, path);
        if(!path.empty()) {
            auto it = path.begin();
            Vec2i prev = *it;
            ++it;
            for(; it != path.end(); ++it) {
                Vec2i current = *it;
                double segment_length = sqrt(pow(current.x - prev.x, 2) + pow(current.y - prev.y, 2));
                path_length_meters += segment_length * aCSpaceMsg.info.resolution;
                prev = current;
            }
        } else {
            double heuristic_distance = sqrt(Distance(next_rendezvous_occ_pos, aOccPos));
            path_length_meters = heuristic_distance * expected_heuristic_error * aCSpaceMsg.info.resolution;
        }
        time_to_reach_next_rendezvous = path_length_meters / expected_speed;
        time_diff = aTimeToReachNextRendezvous - aTotalTime;
        heuristic = time_diff - time_to_reach_next_rendezvous;
        ROS_INFO("[Alysson2025Node] Plan execution monitor \n\n"
                   "\t\t[Next rendezvous location]: (%.2f, %.2f)\n"
                   "\t\t[My current location]: (%.2f, %.2f)\n"
                   "\t\t[Route distance to next rendezvous]: %.2f m\n"
                   "\t\t(reach)[Must arrive at next rendezvous at second]: %.2f\n"
                   "\t\t(mission)[Current mission time]: %.2f s\n"
                   "\t\t(Estimate)[Estimate to reach next rendezvous from my location]: %.2f s\n"
                   "\t\t[My expected velocity]: %.2f m/s\n"
                   "\t\t[Real average velocity]: %.2f m/s\n"
                   "\t\t[Time left (reach - mission)]: %.2f s\n"
                   "\t\t[Control (Estimate - Time left)]: %.2f\n",
                   aGoalRendezvous.getX(), 
                   aGoalRendezvous.getY(),
                   aWorldPos.getX(),
                   aWorldPos.getY(),
                   path_length_meters,
                   aTimeToReachNextRendezvous, 
                   aTotalTime, 
                   time_to_reach_next_rendezvous, 
                   expected_speed,
                   aAverageVelocity,
                   time_diff,
                   heuristic);

        if(heuristic <= 0.0) {
            aPlan->SetCurrentTime(-1.0);
            ROS_INFO("[Alysson2025Node] Resetting timer to -1.0, this will force the robot to wait at rendezvous location.");
        }
    }

    // follow the rendezgous policy only if
    // has something to explore, otherwise,
    // let robots go back to base
    if(aPlan->ShouldFulfillAgreement() && 
        aCurrentState < state_set_rendezvous_location &&
        aCurrentState != state_idle &&
        FinishedMission() == false) {
        ChangeState(state_set_rendezvous_location);
    } 

    // update rendezvous plan only if started
    // the mission
    if(aCurrentState != state_idle && !aFirst) {
        aTotalTime = ros::Time::now().toSec() - aStartingTime;
        aPlan->Update(aDeltaTime);
    }

    // run spin to get the data
    aLastTime = ros::Time::now();

    if(aFirst) aFirst = false;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "Alysson2025Node");
    std::unique_ptr<Alysson2025Node> alysson2025Node = std::make_unique<Alysson2025Node>();
    ros::spin();
}