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

/*
 * Ros and system
 */
#include <vector>
#include <random>
#include "ros/ros.h"

/*
 * Messages
 */
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "std_msgs/Int8MultiArray.h"
#include "multirobotsimulations/CustomPose.h"
#include "multirobotsimulations/Frontiers.h"
#include "visualization_msgs/Marker.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "actionlib/client/simple_action_client.h"
#include "std_msgs/Int32.h"

/*
 * Helpers
 */
#include "Common.h"

typedef enum {
    state_select_frontier = 2,
    state_exploring = 3,
    state_idle = 11,
    state_exploration_finished = 12,
    state_back_to_base = 25,
    state_back_to_base_finished = 26,
    state_compute_centroids = 27,
    state_waiting_centroids = 28,
    state_set_back_to_base = 30,
    state_planning = 31,
} ExplorerState;

class RandomizedSocialWelfareNode {
    public:
        RandomizedSocialWelfareNode();
        ~RandomizedSocialWelfareNode();

    private:
        void EstimatePoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void ClustersCallback(multirobotsimulations::Frontiers::ConstPtr msg);
        void SubGoalFinishCallback(std_msgs::String::ConstPtr msg);
        void CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        void SetExploringCallback(std_msgs::String::ConstPtr msg);
        void SetBasestationCallback(std_msgs::String::ConstPtr msg);
        void SetIdleCallback(std_msgs::String::ConstPtr msg);
        void CommunicationsDirty();
        void CommEvent(std_msgs::Int32::ConstPtr msg);

        void ChangeState(const ExplorerState& newState);
        int SelectFrontier(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld);
        void CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq);
        void SetGoal(const tf::Vector3& goal);       
        void Update();


        void DoneCallback(const actionlib::SimpleClientGoalState& state,
                          const move_base_msgs::MoveBaseResultConstPtr& result);
        void ActiveCallback();
        void FeedbackCallback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback);

        /*
         * Control variables
         */
        int aQueueSize;
        int aRate;
        int aRobots;
        int aId;
        bool aHasPose;
        bool aHasOcc;
        bool aDirty;
        bool aFirst;
        double aDeltaTime;
        Vec2i aOccPos;
        ros::Time aLastTime;
        tf::Vector3 aWorldPos;
        tf::Vector3 aGoalFrontier;
        tf::Vector3 aGoalBasestation;
        std::string aNamespace;
        ExplorerState aCurrentState;
    
        /*
         * Routines
         */
        std::vector<ros::Timer> aTimers;

        /*
         * Subscribers
         */
        std::vector<ros::Subscriber> aSubscribers;
        
        /*
         * Advertisers
         */   
        ros::Publisher aGoalPublisher;
        ros::Publisher aFrontierComputePublisher;

        /*
         * Messages
         */
        multirobotsimulations::Frontiers aFrontierCentroidsMsg;
        nav_msgs::OccupancyGrid aCSpaceMsg;
        std::shared_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> aMoveBaseClient;

        /*
         * Extension from Yamauchi-based policy
         */
        bool CheckNear();
        int RandomizedFrontierSelection(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld);
        void CommCallback(std_msgs::Int8MultiArray::ConstPtr msg);
        
        // extension control
        bool aHasComm;
        Vec2i aFrontierOcc;

        // extension messages
        std_msgs::Int8MultiArray aCommMsg;   

        // extension helpers
        std::unique_ptr<std::mt19937> aRandomNumberGenerator;
        std::random_device aRandomNumberDevice;
};