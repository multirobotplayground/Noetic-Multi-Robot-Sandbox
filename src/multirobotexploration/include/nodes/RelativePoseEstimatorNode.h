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