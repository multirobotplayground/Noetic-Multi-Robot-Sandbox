/*
 * Copyright (c) 2023, Alysson Ribeiro da Silva
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
#include <stdio.h>
#include <queue>
#include "ros/ros.h"
#include "tf/tf.h"

/*
 * Messages
 */
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "multirobotsimulations/CustomPose.h"

/*
 * Helpers
 */
#include "SearchAlgorithms.h"
#include "Common.h"

typedef enum {
    state_idle = 0,
    state_executing_path = 1
} SubGoalState;

class IntegratedGlobalPlannerNode {
    public:
        IntegratedGlobalPlannerNode();
        ~IntegratedGlobalPlannerNode();

    private:
        void CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq);
        void DepthFirstSearchFreePath(
            nav_msgs::OccupancyGrid& cspace, 
            Vec2i& occpos,
            Vec2i& source, 
            Vec2i& closest,
            std::list<Vec2i>& outpath);
        void ChangeState(const SubGoalState& newState);
        void CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        void PoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void AverageVelocityCallback(std_msgs::Float32::ConstPtr msg);
        void GoalCallback(geometry_msgs::Pose::ConstPtr msg);
        void StopCallBack(std_msgs::String::ConstPtr msg);
        void Update();

        /*
         * Control variables
         */
        int aQueueSize;
        int aId;
        int aSeq;
        int aDeltaTimeSec;
        bool aHasPose;
        bool aHasOcc;
        bool aHasAverageVelocity;
        double aRate;
        double aDistance;
        double aSubGoalReachThreshold;
        double aStuckTimeThreshold;
        double aStuckTime;
        double aAverageVelocity;
        std::string aNamespace;

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
        ros::Publisher aPathMarkerPublisher;
        ros::Publisher aFinishEventPublisher;
        ros::Publisher aCurrentPathPublisher;

        /*
         * Messages
         */
        nav_msgs::Path aPathMsg;
        std_msgs::String aStrMsg;
        visualization_msgs::Marker aPathMarkerMsg;
        nav_msgs::OccupancyGrid aCspace;

        /*
         * Helpers
         */
        ros::Time last_time;
        std::list<Vec2i> aWaypoints;
        tf::Vector3 aLastPos;
        tf::Vector3 aWorldPos;
        tf::Vector3 aCurrentGoal;
        Vec2i aOccPos;
        SubGoalState aCurrentState;
};