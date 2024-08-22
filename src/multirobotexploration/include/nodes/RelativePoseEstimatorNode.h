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
 * ROS and system
 */
#include <stdio.h>
#include <vector>
#include "ros/ros.h"
#include "tf/tf.h"

/*
 * Messages
 */
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "multirobotsimulations/CustomPose.h"
#include "multirobotsimulations/MockPackage.h"

/*
 * Helpers
 */
#include "Common.h"

class RelativePoseEstimatorNode {
    public:
        RelativePoseEstimatorNode();
        ~RelativePoseEstimatorNode();

    private:
        void LoadRelativePoses(ros::NodeHandle& nodeHandle);
        void PrepareMarkers(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq);
        void SetNear(visualization_msgs::Marker& input);
        void SetFar(visualization_msgs::Marker& input);
        void CommCallback(std_msgs::Int8MultiArray::ConstPtr msg);
        void Update();

        /*
         * Control variables
         */
        int aRobots;
        int aId;
        int aQueueSize;
        int aSeq;
        double aRate;
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
        ros::Publisher aStartRelativePosesPublisher;
        ros::Publisher aRelativePosesPublisher;
        ros::Publisher aDistancesPublisher;
        ros::Publisher aNearMarkerPublisher;
        ros::Publisher aFarMarkerPublisher;

        /*
         * Messages
         */
        geometry_msgs::PoseArray aRobotsRelStartingPosMsg;
        geometry_msgs::PoseArray aRobotsRelativePosesMsg;
        visualization_msgs::Marker aClusterMarkerMsg;
        visualization_msgs::Marker aClusterMarkerFarMsg;
        std_msgs::Float64MultiArray aRobotsRelativeDistancesMsg;
        std_msgs::Int8MultiArray aRobotsInCommMsg;

        /*
         * Helpers
         */
        std::vector<geometry_msgs::Pose> aRobotsWorldPoses;
        std::vector<bool> aReceivedPoses;
        std::vector<tf::Vector3> aRelativePoses;
};