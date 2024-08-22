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
#include <iostream>
#include <stdio.h>
#include <string.h>
#include "ros/ros.h"
#include "tf2_ros/transform_listener.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/tf.h"

/*
 * Messages
 */
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Point.h"
#include "nav_msgs/MapMetaData.h"
#include "nav_msgs/OccupancyGrid.h"
#include "multirobotsimulations/CustomPose.h"
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"

/*
 * Helpers
 */
#include "Common.h"

class GmappingPoseNode {
    public:
        GmappingPoseNode();
        ~GmappingPoseNode();

    private:
        void OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        void Update();

        /*
         * Control variables
         */
        int aId;
        int aQueueSize;
        bool aHasOcc;
        double aRate;
        std::string aNamespace;
        std::string aTFBaseLink;
        std::string aTFMap;

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
        ros::Publisher aPosePublisher;
        ros::Publisher aPoseStampedPublisher;
        ros::Publisher aPathPublisher;

        /*
         * Messages
         */
        nav_msgs::OccupancyGrid aOcc;
        multirobotsimulations::CustomPose aPose;
        geometry_msgs::PoseStamped aPoseStamped;
        nav_msgs::Path aPath;

        /*
         * Helpers
         */
        std::shared_ptr<tf2_ros::Buffer> aTFBuffer;
        std::shared_ptr<tf2_ros::TransformListener> aTFListener;

};
