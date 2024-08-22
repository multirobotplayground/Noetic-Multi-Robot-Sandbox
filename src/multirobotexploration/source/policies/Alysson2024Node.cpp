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

#include "Alysson2024Node.h"

Alysson2024Node::Alysson2024Node() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots.");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("waiting_threshold", aWaitingThreshold)) aWaitingThreshold = 120.0;

    std::map<std::string, double> pose;
    if(!node_handle.getParam("/first_rendezvous", pose)) throw std::runtime_error("Could not retrieve /first_rendezvous");
    aFirstRendezvous.setX(pose["x"]);
    aFirstRendezvous.setY(pose["y"]);
    aFirstRendezvous.setZ(pose["z"]);
    ROS_INFO("[Alysson2024Node] First rendezvous location: %f %f %f", pose["x"], pose["y"], pose["z"]); 

    std::string key = "/footprint_robot_" + std::to_string(aId);
    if(!node_handle.getParam(key, pose)) throw std::runtime_error("Could not retrieve /footprint_robot_" + std::to_string(aId));
    aRendezvousFootprint.setX(pose["x"]);
    aRendezvousFootprint.setY(pose["y"]);
    aRendezvousFootprint.setZ(pose["z"]);
    ROS_INFO("[Alysson2024Node] Rendezvous footprint: %f %f %f", pose["x"], pose["y"], pose["z"]);

    aNamespace = ros::this_node::getNamespace();

    aHasOcc = false;
    aHasPose = false;
    aHasComm = false;
    aFirst = true;
    aDirty = true;
    aTimeWaiting = 0.0;
    aRendezvousMsg.robot_id = aId;
    aRendezvousNewPoseMsg.robot_id = aId;
    aCurrentState = state_idle;

    // initialize containers
    aRandomNumberGenerator = std::make_unique<std::mt19937>(aRandomNumberDevice());
    aPlan = std::make_shared<RendezvousPlan>(node_handle, aId);

    // initialize rendezvous plan
    aPlan->InitializeLocation(aFirstRendezvous);
    aPlan->PrintLocations();
    aPlan->Print();

    // Subscriptions
    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::Frontiers>(
            aNamespace + "/frontier_discovery/frontiers_clusters", 
            aQueueSize, std::bind(&Alysson2024Node::ClustersCallback, this, std::placeholders::_1)));
    
    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::CustomPose>(
            aNamespace + "/gmapping_pose/world_pose", 
            aQueueSize, 
            std::bind(&Alysson2024Node::EstimatePoseCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/integrated_global_planner/finish", 
            aQueueSize, 
            std::bind(&Alysson2024Node::SubGoalFinishCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<nav_msgs::OccupancyGrid>(
            aNamespace + "/c_space", 
            aQueueSize, 
            std::bind(&Alysson2024Node::CSpaceCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/explorer/set_idle", 
            aQueueSize, 
            std::bind(&Alysson2024Node::SetIdleCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            aNamespace + "/explorer/set_exploring", 
            aQueueSize, 
            std::bind(&Alysson2024Node::SetExploringCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            "/global_explorer/back_to_base", 
            aQueueSize, 
            std::bind(&Alysson2024Node::SetBasestationCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::String>(
            "/global_explorer/set_exploring", 
            aQueueSize, 
            std::bind(&Alysson2024Node::SetExploringCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Int8MultiArray>(
            aNamespace + "/mock_communication_model/robots_in_comm", 
            aQueueSize,
            std::bind(&Alysson2024Node::CommCallback, this, std::placeholders::_1)));

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
                    if(msg->plan == planPtr->GetCurrentAgreementUniqueID() && CanCommunicate(msg->robot_id))
                        planPtr->RealizePlan(msg->robot_id);
                }
            ));
        
        aSubscribers.push_back(
            node_handle.subscribe<multirobotsimulations::CustomPose>(
                "/robot_" + std::to_string(robot) + "/plan_updater",
                aQueueSize,
                [planPtr, commPtr, receivedRendezvousLocationPtr, newRendezvousLocationPtr, this](multirobotsimulations::CustomPose::ConstPtr msg) {
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
    }

    // Advertisers
    aGoalPublisher = node_handle.advertise<geometry_msgs::Pose>(aNamespace + "/integrated_global_planner/goal", aQueueSize);
    aFrontierComputePublisher = node_handle.advertise<std_msgs::String>(aNamespace + "/frontier_discovery/compute", aQueueSize);
    aPlanRealizationPublisher = node_handle.advertise<multirobotsimulations::rendezvous>(aNamespace + "/realizing_plan", aQueueSize);
    aPlanLocationPublisher = node_handle.advertise<multirobotsimulations::CustomPose>(aNamespace + "/plan_updater", aQueueSize);

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&Alysson2024Node::Update, this)));
}

Alysson2024Node::~Alysson2024Node() {

}

void Alysson2024Node::ClustersCallback(multirobotsimulations::Frontiers::ConstPtr msg) {
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
    if(aCurrentState == state_waiting_centroids_for_plan) ChangeState(state_select_new_rendezvous);
}

void Alysson2024Node::EstimatePoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    if(!aHasPose) aHasPose = true;
    aWorldPos.setX(msg->pose.position.x);
    aWorldPos.setY(msg->pose.position.y);
}

void Alysson2024Node::SubGoalFinishCallback(std_msgs::String::ConstPtr msg) {
    if(aCurrentState == state_exploring) ChangeState(state_exploration_finished);
    if(aCurrentState == state_back_to_base) ChangeState(state_back_to_base_finished);
    if(aCurrentState == state_navigating_to_rendezvous) ChangeState(state_at_rendezvous);
}

void Alysson2024Node::CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg) {
    if(!aHasOcc) aHasOcc = true;
    aCSpaceMsg.info = msg->info;
    aCSpaceMsg.header = msg->header;
}

void Alysson2024Node::SetIdleCallback(std_msgs::String::ConstPtr msg) {
    ChangeState(state_idle);
}

void Alysson2024Node::SetBasestationCallback(std_msgs::String::ConstPtr msg) {
    ChangeState(state_set_back_to_base);
}

void Alysson2024Node::SetExploringCallback(std_msgs::String::ConstPtr msg) { 
    ChangeState(state_compute_centroids);
}

int Alysson2024Node::SelectFrontier(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld) {
    selectFrontierWorld.setX(centroids.centroids.poses[centroids.highest_utility_index].position.x);
    selectFrontierWorld.setY(centroids.centroids.poses[centroids.highest_utility_index].position.y);
    return centroids.highest_utility_index;
}

void Alysson2024Node::CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq) {
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

void Alysson2024Node::SetGoal(const tf::Vector3& goal) {
    geometry_msgs::Pose pose_msg;
    pose_msg.position.x = goal.getX();
    pose_msg.position.y = goal.getY();
    aGoalPublisher.publish(pose_msg);    
}

void Alysson2024Node::ChangeState(const ExplorerState& newState) {
    ROS_INFO("[Alysson2024Node] State change %d -> %d.", aCurrentState, newState);
    aCurrentState = newState;
}

bool Alysson2024Node::CheckNear() {
    for(size_t robot = 0; robot < aCommMsg.data.size(); ++robot) {
        if(robot == aId) continue;
        if(aCommMsg.data[robot] == 1) return true;
    }
    return false;
}

bool Alysson2024Node::CanCommunicate(const int& id) {
    if(id < 0 || id >= aCommMsg.data.size()) throw std::out_of_range("Robot id is out of range in CanCommunicate.");
    if(aCommMsg.data[id] == 1) return true;
    return false;   
}

bool Alysson2024Node::FinishedMission() {
    return (aCurrentState == state_back_to_base ||
            aCurrentState == state_back_to_base_finished ||
            aCurrentState == state_set_back_to_base);
}

int Alysson2024Node::RandomizedFrontierSelection(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld) {
    // simply creates a distribution and samples a position from the centroids array
    std::uniform_int_distribution<int> distribution(0, centroids.centroids.poses.size()-1);
    int selected = distribution(*aRandomNumberGenerator); 
    selectFrontierWorld.setX(centroids.centroids.poses[selected].position.x);
    selectFrontierWorld.setY(centroids.centroids.poses[selected].position.y);
    return selected;  
}

int Alysson2024Node::SelectSubteamNewRendezvous(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld) {
    selectFrontierWorld.setX(centroids.centroids.poses[centroids.highest_cost_index].position.x);
    selectFrontierWorld.setY(centroids.centroids.poses[centroids.highest_cost_index].position.y);
    return centroids.highest_cost_index;
}

void Alysson2024Node::CommCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    if(!aHasComm) aHasComm = true;
    aCommMsg.data.assign(msg->data.begin(), msg->data.end());
    aCommMsg.layout = msg->layout;
}

void Alysson2024Node::Update() {
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
                ROS_INFO("[Alysson2024Node] Not more clusters to explore [%.2f %.2f]", 
                            aGoalBasestation.getX(), 
                            aGoalBasestation.getY());
                break;
            }
            
            if(aFrontierCentroidsMsg.centroids.poses.size() > 0) {
               if(CheckNear()) {
                    RandomizedFrontierSelection(aFrontierCentroidsMsg, aGoalFrontier);
                    ROS_INFO("[Alysson2024Node] randomized utility.");
                } else {
                    SelectFrontier(aFrontierCentroidsMsg, aGoalFrontier);
                    ROS_INFO("[Alysson2024Node] maximizing utility.");
                }
                ROS_INFO("[Alysson2024Node] selected frontier [%.2f %.2f]", 
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
            ROS_INFO("[Alysson2024Node] going back to base at [%.2f %.2f]", 
                        aGoalBasestation.getX(), 
                        aGoalBasestation.getY());   
            ChangeState(state_back_to_base);             
        break;

        case state_back_to_base:

        break;

        case state_back_to_base_finished:
            ROS_INFO("[Alysson2024Node] reached motherbase.");
            ChangeState(state_idle);
        break;

        case state_exploration_finished:
            ROS_INFO("[Alysson2024Node] reached frontier.");
            ChangeState(state_compute_centroids);
        break;
        
        case state_set_rendezvous_location:
            aGoalRendezvous = aPlan->GetCurrentAgreementLocation();

            // consider the footprint for this robot
            // this was one of the most efficient ways to avoid
            // collision
            aGoalRendezvous += aRendezvousFootprint;

            SetGoal(aGoalRendezvous);
            ROS_INFO("[Alysson2024Node] fulfilling plan %d of unique id %d",
                        aPlan->GetCurrentAgreement(), 
                        aPlan->GetCurrentAgreementUniqueID());
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
                if(aPlan->CheckConsensusCurrentPlan() == true) {
                    ChangeState(state_updating_plan);
                    ROS_INFO("[Alysson2024Node] I am consensus robot.");
                    aTimeWaiting = 0.0;
                } else {
                    ChangeState(state_waiting_consensus);
                    ROS_INFO("[Alysson2024Node] Waiting consensus.");
                    aTimeWaiting = 0.0;
                }
            } else {
                ROS_INFO("[Alysson2024Node] Waiting at rendezvous for %fs", aTimeWaiting);

                if(aTimeWaiting > aWaitingThreshold) {
                    ChangeState(state_compute_centroids);
                    aTimeWaiting = 0.0;
                    aPlan->ResetPlanRealization();
                    ROS_INFO("[Alysson2024Node] No one went to rendezvous, reseting this plan.");
                }
            }

            aTimeWaiting += aDeltaTime;
        break;

        case state_updating_plan:
            // ask for centroids to avoid
            // unnecessary computations
            aFrontierComputePublisher.publish(aFrontierCentroidsMsg);
            ChangeState(state_waiting_centroids_for_plan);
            ROS_INFO("[Alysson2024Node] Consensus asking for frontiers to update rendezvous location.");
        break;

        case state_waiting_centroids_for_plan:
            // just wait
        break;

        case state_select_new_rendezvous:
            // select the fartest cluster as a new rendezvous location
            // this forces the robots to explore more near unknown areas
            if(aFrontierCentroidsMsg.centroids.poses.size() > 0)
                ROS_INFO("[Alysson2024Node] Selected rendezvous index: %d", SelectSubteamNewRendezvous(aFrontierCentroidsMsg, aGoalFrontier));

            // aways send updated plan to unstuck 
            // in situations where there are no more frontiers
            // to explore
            aPlan->UpdateCurrentAgreementLocation(aGoalFrontier);
            aPlan->ResetPlanRealization();
            aPlan->SetNextAgreement();

            ROS_INFO("[Alysson2024Node] New location selected [%f %f], sending to others.",
                aGoalFrontier.getX(), aGoalFrontier.getY());

            // send the new location to the other robots and start
            // exploring again
            aRendezvousNewPoseMsg.pose.position.x = aGoalFrontier.getX();
            aRendezvousNewPoseMsg.pose.position.y = aGoalFrontier.getY();
            aRendezvousNewPoseMsg.pose.position.z = 0.0;
            aPlanLocationPublisher.publish(aRendezvousNewPoseMsg);

            ChangeState(state_select_frontier);
        break;

        case state_waiting_consensus:                
            // waiting for new rendezvous location from the
            // consensus robot
            if(aReceivedNewRendezvousLocation) {
                aPlan->UpdateCurrentAgreementLocation(aNewRendezvousLocation);
                aPlan->ResetPlanRealization();
                aPlan->PrintCurrent();
                ROS_INFO("[Alysson2024Node] received new rendezvous location [%f %f].",
                    aNewRendezvousLocation.getX(), 
                    aNewRendezvousLocation.getY());
                aPlan->SetNextAgreement();
                ROS_INFO("[Alysson2024Node] the new agreement to be fulfilled was updated to.");
                aPlan->PrintCurrent();
                ChangeState(state_compute_centroids);
                aReceivedNewRendezvousLocation = false;
                aTimeWaiting = 0.0;
            } else {
                ROS_INFO("[Alysson2024Node] Waiting for new rendezvous location for %fs", aTimeWaiting);

                if(aTimeWaiting > aWaitingThreshold) {
                    aPlan->ResetPlanRealization();
                    aPlan->SetNextAgreement();
                    ChangeState(state_compute_centroids);
                    ROS_INFO("[Alysson2024Node] Consensus seems to have died, reseting to next plan.");
                    aTimeWaiting = 0.0;
                }
            }

            aTimeWaiting += aDeltaTime;
        break;
    }

    // update rendezvous plan only if started
    // the mission
    if(aCurrentState != state_idle && !aFirst)
        aPlan->Update(aDeltaTime);

    if(aCurrentState < state_set_rendezvous_location &&
        aCurrentState != state_idle) {
        aPlan->PrintCurrent();   
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

    aDeltaTime = ros::Time::now().sec - aLastTime.sec;
    ROS_INFO("[Alysson2024Node] current state %d delta time: %f", aCurrentState, aDeltaTime);

    // run spin to get the data
    aLastTime = ros::Time::now();

    if(aFirst) aFirst = false;
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "alysson2024node");
    std::unique_ptr<Alysson2024Node> alysson2024Node = std::make_unique<Alysson2024Node>();
    ros::spin();
}