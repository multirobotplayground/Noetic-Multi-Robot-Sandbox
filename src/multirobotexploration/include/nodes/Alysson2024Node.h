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
#include "multirobotsimulations/rendezvous.h"
#include "visualization_msgs/Marker.h"

/*
 * Helpers
 */
#include "RendezvousPlan.h"
#include "Common.h"

/*! Robot states. */
typedef enum {
    state_select_frontier = 2, /**< state responsible to select which frontier cluster to explore. */
    state_exploring = 3, /**< state responsible to explore a selected location. */
    state_idle = 11, /**< in this state the robot waits for commands.*/
    state_exploration_finished = 12, /**< this states marks the end of the mission when all frontiers were explored. */
    state_back_to_base = 25, /**< this state tells that the robot is going to the initial basestation location. */
    state_back_to_base_finished = 26, /**< this state is used to identify when the robot reached the basestation. */
    state_compute_centroids = 27, /**< used to wait for the FrontierDiscovery to return frontier clusters. */
    state_waiting_centroids = 28, /**< in this state, the robot waits for frontier clusters. */
    state_set_back_to_base = 30, /**< in this state, the robot navigates towards the basestation. */
    state_planning = 31, /**< in this state, the robot asks for new frontier clusters and also selects which one to explore. */
    
    /*
     * Extended state space
     */ 
    state_set_rendezvous_location = 55, /**< in this state, the robot starts navigating towards its current sub-team rendezvous location. */
    state_navigating_to_rendezvous = 56, /**< in this state, the robot simply navigate towards the assigned rendezvous location. */
    state_at_rendezvous = 57, /**< in this state, the robot reached the rendezvous location and can start updating the plan or waiting for the consensus. */
    state_updating_plan = 58, /**< in this state, the consensus robot updates the plan and send to the other robots. */
    state_waiting_consensus = 59, /**< in this state, the other robots wait for the information from the consensus robot. */
    state_waiting_centroids_for_plan = 60, /**< in this state the consensus robot waits for clusters to use as reference updates for new rendezvous locations. */
    state_select_new_rendezvous = 61 /**< in this state, the consensus robot sends the new rendezvous locations to all other robots. */
} ExplorerState;

/**
 * Node that implements the intermittent communication policy. While exploring,
 * robots are going to meet at rendezvous locations spread dynamically while exploration
 * happens. The joint-policy spreads robots when they can communicate and maximize
 * the utility of visiting frontiers when they are alone. Robots also share maps
 * when near each other considering a mock communication model based on communication range.
 */
class Alysson2024Node {
    public:
        /**
         * Default constructor that initializes all ros advertisers, publishers, and routines.
         */
        Alysson2024Node();

        /**
         * Default destructor.
         */
        ~Alysson2024Node();

    private:
        /**
         * Callback for pose estimator from the SLAM algorithm.
         * @param msg multirobotsimulations::CustomPose.
         */
        void EstimatePoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);

        /**
         * Callback for the FrontierDiscovery node to update rendezvous locations or for planning.
         * @param msg multirobotsimulations::Frontiers.
         */
        void ClustersCallback(multirobotsimulations::Frontiers::ConstPtr msg);

        /**
         * Callback from the IntegratedGlobalPlanner node that is called when the robot reached the assigned goal.
         * @param msg std_msgs::String.
         */
        void SubGoalFinishCallback(std_msgs::String::ConstPtr msg);

        /**
         * Callback from the MapStitching node that is used to receive meta data from the map.
         * @param msg nav_msgs::OccupancyGrid.
         */
        void CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg);

        /**
         * Callback this node uses to change its internal state to start exploring.
         * @param msg std_msgs::String message received.
         */
        void SetExploringCallback(std_msgs::String::ConstPtr msg);

        /**
         * Callback this node uses to go back to the basestation.
         * @param msg std_msgs::String message received.
         */
        void SetBasestationCallback(std_msgs::String::ConstPtr msg);

        /**
         * Callback this node uses to stop the mission and enter in idle mode.
         * @param msg std_msgs::String message received.
         */
        void SetIdleCallback(std_msgs::String::ConstPtr msg);

        /** 
         * Change the state of the robot and print information about the change.
         * @param newState integer referencing the state to change to.
         */
        void ChangeState(const ExplorerState& newState);

        /**
         * Select which frontier to explore from the message received by the FrontierDiscovery node.
         * The best frontier, the one that maximizes the utility one time step into the future, is already 
         * computed by the FrontierDiscovery node, thus saving some processing.
         * @param centroids multirobotsimulations::Frontiers message holding the available frontier clusters.
         * @param selectFrontierWorld tf::Vector3 referencee to the selected cluster in the world's reference frame.
         * @return integer index of the frontier cluster that maximizes the utility one time step into the future.
         */
        int SelectFrontier(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld);

        /**
         * Create a marker to mark selected goal locations (e.g., rendezvous locations or frontier clusters).
         * @param input visualization_msgs::Marker of object from ROS to initialize.
         * @param ns const char* specifying the namespace of this robot to assemble the reference frame name.
         * @param id integer holding the unique id of this robot to assemble the reference frame name.
         * @param seq integer holding the current marker index.
         */
        void CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq);

        /**
         * Set the goal location to visit, it could be a rendezvousl location or a frontier cluster center of mass.
         * @param goal tf::Vector3 holding the goal location in the world reference frame.
         */
        void SetGoal(const tf::Vector3& goal);       

        /**
         * Node's main update routine. It runs the robot's state machine or policy.
         */
        void Update();

        /*
         * Control variables
         */
        int aQueueSize; /**< Size of all publishers and subscribers queues. */
        int aRate; /**< Main loop rate. */
        int aRobots; /**< Number of robots this robot should be aware. */
        int aId; /**< Unique id. */
        bool aHasPose; /**< Used to known when this robot received its estimate pose from the SLAM sub-system. */
        bool aHasOcc; /**< Used to known when this robot received its OccupancyGrid information from the SLAM sub-system. */
        bool aDirty; /**< Used to initialize the basestation location in the main loop only once. */
        bool aFirst; /**< Used to skip the first delta time calculation to avoid major errors. */
        double aDeltaTime; /**< Delta time between iterations in the main loop. */
        Vec2i aOccPos; /**< The position of the robot in the Occupancy Grid reference frame. */
        ros::Time aLastTime; /**< The last capatured system time used to compute the dalta time. */
        tf::Vector3 aWorldPos; /**< The position of the robot in the world reference frame. */
        tf::Vector3 aGoalFrontier; /**< Temp variable used to store which frontier this robot should explore next */
        tf::Vector3 aGoalBasestation; /**< The goal position of the basestation. */
        std::string aNamespace; /**< The namespace of this robot. */
        ExplorerState aCurrentState; /**< The current state of the robot. */

        /*
         * Routines
         */
        std::vector<ros::Timer> aTimers; /**< Routines of this node. They are stored in a list format for simplicity. */

        /*
         * Subscribers
         */
        std::vector<ros::Subscriber> aSubscribers; /**< All subscribers of this node. Since the subscriber object is not used throughout the node, it can be stored as an array for cleaner code. */
        
        /*
         * Advertisers
         */   
        ros::Publisher aGoalPublisher; /**< This publisher is used to tell the global planner, which goal location the robot should visit next. */
        ros::Publisher aFrontierComputePublisher; /**< This publisher is used to request the FrontierDiscovery module to compute new frontier clusters. */

        /*
         * Messages
         */
        multirobotsimulations::Frontiers aFrontierCentroidsMsg; /**< Message used to receive frontier clusters. */
        nav_msgs::OccupancyGrid aCSpaceMsg; /**< Message used to store meta data from the occupancy grid to help converting world positions to positions in the grid. */

        /*
         * Extension from Randomized-Social-Welfare policy
         */

        /**
         * Used to help checking if there are other robots nearby. This information is used to 
         * make decisions, such as where to explore.
         */
        bool CheckNear();

        /**
         * Used to check if this robot can communicate with another.
         * @param id integer specifying the id of the other robot.
         * @return false if it cannot communicate with the other robot, returns true otherwise.
         */
        bool CanCommunicate(const int& id);

        /**
         * Check if there are places left to explore.
         * @return false if there are places to explore, true otherwise.
         */
        bool FinishedMission();

        /**
         * Select a randomized frontier.
         * @param centroids multirobotsimulations::Frontiers message with all frontier clusters, their utilities, and best options.
         * @param selectFrontierWorld tf::Vector3 selected cluster.
         * @return integer index of the selected cluster.
         */
        int RandomizedFrontierSelection(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld);

        /**
         * Select a new rendezvous location that is the frontier with the highest cost (i.e., that spread the robots the most).
         * @param centroids multirobotsimulations::Frontiers message with all frontier clusters, their utilities, and best options.
         * @param selectFrontierWorld tf::Vector3 selected cluster.
         * @return integer index of the selected cluster.
         */
        int SelectSubteamNewRendezvous(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld);

        /**
         * Callback of the MockCommunication model used to check which robots are nearby.
         * @param msg std_msgs::Int8MultiArray.
         */
        void CommCallback(std_msgs::Int8MultiArray::ConstPtr msg);

        // extension control
        bool aHasComm; /**< Flag used to check if the robot received the mock communication model communications array. */
        bool aReceivedNewRendezvousLocation; /**< Flag used to check if this robot received new rendezvous location in the it is not the consensus robot. */
        double aWaitingThreshold; /**< The maximum allowed waiting time in rendezvous location. If this timer is exceeded when waiting for other robots, then 
                                        this robot should reset the plan and start exploring again.*/
        double aTimeWaiting; /**< Counter to help checking if this robot is waiting for too long. */
        std::shared_ptr<RendezvousPlan> aPlan; /**< The rendezvous plan object used to control how agreements are executed, their order, and which ones to update. */

        // extension messages
        std_msgs::Int8MultiArray aCommMsg; /**< Message to receive information from the mock communication model node. */
        multirobotsimulations::rendezvous aRendezvousMsg; /**< Message that helps the consensus robot sending new rendezvous locations to other robots. */ 
        multirobotsimulations::CustomPose aRendezvousNewPoseMsg; /**< Custom pose used to tell the other robots which location they should meet next with their current sub-team. */

        // extension helpers
        tf::Vector3 aRendezvousFootprint; /**< The rendezvous location this robot should go when inside a rendezous zone. */
        tf::Vector3 aFirstRendezvous; /**< Store the location of the first rendezvous location for all sub-teams. */
        tf::Vector3 aGoalRendezvous; /**< Helper variable that holds the location of the rendezvous location. */
        tf::Vector3 aNewRendezvousLocation; /**< Helper variable used to keep track where this robot should meet next with its current sub-team.*/
        std::unique_ptr<std::mt19937> aRandomNumberGenerator; /**< Random number generator for randomized social welfare. */
        std::random_device aRandomNumberDevice; /**< Random device to use with std::mt19937 to generate random numbers.*/

        // extension advertisers
        ros::Publisher aPlanRealizationPublisher; /**< Publisher used to help synchronize who is fulfilling a rendezvous agreement. */
        ros::Publisher aPlanLocationPublisher; /**< Publisher to broadcast new rendezvous locations for the current sub-team. */
};