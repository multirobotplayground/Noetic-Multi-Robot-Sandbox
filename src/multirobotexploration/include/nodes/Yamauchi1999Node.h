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
#include "ros/ros.h"

/*
 * Messages
 */
#include "nav_msgs/OccupancyGrid.h"
#include "std_msgs/String.h"
#include "multirobotsimulations/CustomPose.h"
#include "multirobotsimulations/Frontiers.h"
#include "visualization_msgs/Marker.h"

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

class Yamauchi1999Node {
    public:
        Yamauchi1999Node();
        ~Yamauchi1999Node();

    private:
        void EstimatePoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void ClustersCallback(multirobotsimulations::Frontiers::ConstPtr msg);
        void SubGoalFinishCallback(std_msgs::String::ConstPtr msg);
        void CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        void SetExploringCallback(std_msgs::String::ConstPtr msg);
        void SetBasestationCallback(std_msgs::String::ConstPtr msg);
        void SetIdleCallback(std_msgs::String::ConstPtr msg);

        void ChangeState(const ExplorerState& newState);
        int SelectFrontier(multirobotsimulations::Frontiers& centroids, tf::Vector3& selectFrontierWorld);
        void CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq);
        void SetGoal(const tf::Vector3& goal);       
        void Update();

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
};