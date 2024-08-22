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
 * Ros and systems
 */
#include <string>
#include <ros/ros.h>
#include <signal.h>
#include "tf/tf.h"
#include "string.h"

/*
 * Messages
 */
#include "multirobotsimulations/CustomOcc.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/Pose.h"
#include "costmap_converter/ObstacleArrayMsg.h"
#include "std_msgs/Int8MultiArray.h"
#include "multirobotsimulations/CustomPose.h"

/*
 * Helpers 
 */
#include "Common.h"

/*
 * LocalCSpaceNode class
 */
class LocalCSpaceNode {
    public:
        LocalCSpaceNode();
        ~LocalCSpaceNode();

    private:
        void CreateLocal(nav_msgs::OccupancyGrid& dynamicOcc, 
                            nav_msgs::OccupancyGrid& localMap,
                            geometry_msgs::PoseArray& occupiedPoses,
                            geometry_msgs::PoseArray& freePoses,
                            tf::Vector3& worldPose,
                            tf::Vector3& occPose,
                            const double& freeInflationRadius,
                            const double& occupiedInflationRadius, 
                            const int& windws_size_meters,
                            const int8_t& occupancyThreshold = 90,
                            const int8_t& freeThreshold = 50,
                            const int8_t& occupiedValue = 100,
                            const int8_t& freeVal = 1,
                            const int8_t& unknownVal = -1);

        void ApplyDynamicData(nav_msgs::OccupancyGrid& occ,
                                nav_msgs::OccupancyGrid& dynamicOcc,
                                std::vector<geometry_msgs::PoseArray>& lidarSources,
                                std::vector<geometry_msgs::PoseArray>& otherSources,
                                const double& maxLidarRange = 10.0,
                                const int8_t& occupiedValue = 100);
        void ClearLocalTrajectories(std::vector<geometry_msgs::PoseArray>& local, 
                                    std_msgs::Int8MultiArray& comm);

        void RobotsInCommCallback(std_msgs::Int8MultiArray::ConstPtr msg);
        void OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        void WorldPoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);

        void Update();


        /*
         * Control variables
         */
        int aQueueSize;
        int aId;
        int aLidarSources;
        int aRobots;
        int aLocalViewSize;
        bool aHasOcc;
        bool aHasPose;
        bool aReceivedComm;
        double aRate;
        double aLidarRange;
        double aFreeInflateRadius;
        double aOccuInflateRadius;
        tf::Vector3 aOccPose;
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
        ros::Publisher aObstaclesPublisher;
        ros::Publisher aLocalCSpacePublisher;
        ros::Publisher aOccupiedPositionsPublisher;
        ros::Publisher aFreePositionsPublisher;

        /*
         * Messages
         */
        nav_msgs::OccupancyGrid aOccMsg;
        nav_msgs::OccupancyGrid aOccWithDynamicDataMsg;
        nav_msgs::OccupancyGrid aLocalCspaceMsg;
        std_msgs::Int8MultiArray aRobotsInCommMsg;    
        multirobotsimulations::CustomPose aWorldPoseMsg;
        geometry_msgs::PoseArray aOccupiedPosesMsg;
        geometry_msgs::PoseArray aFreePosesMsg;
        
        /*
         * Helpers
         */
        std::vector<geometry_msgs::PoseArray> aTrajectoriesArray;
        std::vector<geometry_msgs::PoseArray> aLidarsArray;
};
