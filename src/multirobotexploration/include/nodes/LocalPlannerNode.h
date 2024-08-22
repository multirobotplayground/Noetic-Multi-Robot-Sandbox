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
#include "ros/ros.h"
#include "teb_local_planner/teb_local_planner_ros.h"

/*
 * Messages
 */
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32.h"
#include "multirobotsimulations/CustomPose.h"
#include "multirobotsimulations/MockPackage.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseArray.h"

/*
 *
 */
#include "Common.h"

class LocalPlannerNode {
    public:
        LocalPlannerNode();
        ~LocalPlannerNode();

    private:
        void CreateMarker(visualization_msgs::Marker& marker, const char* ns, const int& id, const int& seq);
        void AssembleSparsePath(nav_msgs::Path& currentPath, nav_msgs::Path& filteredPath, const int& viaIncrement, visualization_msgs::Marker& globalPathMaker);
        bool CheckNearPriority();
        void ObstacleArrayCallback(costmap_converter::ObstacleArrayConstPtr msg);
        void PoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void SubgoalPathCallback(nav_msgs::Path::ConstPtr msg);
        void RobotInCommCallback(std_msgs::Int8MultiArray::ConstPtr msg);
        void Update();

        /*
         * Control variables
         */
        int aQueueSize;
        int aSeq;
        int aId;
        int aRobots;
        int aMaxWaypoints;
        int aViaIncrement;
        int aIncrement;
        int aControlsToShare;
        bool aUsePriorityBehavior;
        bool aReceivedComm;
        double aRate;
        std::string aNamespace;
        std::string aName;

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
        ros::Publisher aVelocityPublisher;
        ros::Publisher aTebPosesPublisher;
        ros::Publisher aViaPointsPublisher;
        
        /*
         * Messages
         */
        geometry_msgs::Twist aTwistVelMsg;
        geometry_msgs::PoseStamped aPrevPoseMsg;
        geometry_msgs::PoseStamped aLastPoseMsg;
        geometry_msgs::PoseArray aTebPosesMsg;
        visualization_msgs::Marker aGlobalPathMsg;
        std_msgs::Int8MultiArray aRobotsInCommMsg;
        nav_msgs::Path aCurrentPathMsg;
        nav_msgs::Path aFilteredPathMsg;
        multirobotsimulations::CustomPose aPose;

        /*
         * Helpers
         */
        teb_local_planner::TebConfig aTebConfig;
        teb_local_planner::ViaPointContainer aViaPoints;
        teb_local_planner::TebVisualizationPtr aVisual;
        teb_local_planner::RobotFootprintModelPtr aRobotFootprint;
        std::vector<teb_local_planner::ObstaclePtr> aObstacleArray;
        std::shared_ptr<teb_local_planner::HomotopyClassPlanner> aPlanner;
};