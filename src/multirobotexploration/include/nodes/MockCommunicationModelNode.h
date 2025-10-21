/*
 * Noetic-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
 * Copyright (C) 2023 Alysson Ribeiro da Silva
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
 * System and ROS
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
#include "std_msgs/Int32.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "multirobotsimulations/CustomPose.h"

/*
 * Helpers
 */
#include "Common.h"

class MockCommunicationModelNode {
    public:
        MockCommunicationModelNode();
        ~MockCommunicationModelNode();

    private:
        void LoadRelativePoses(ros::NodeHandle& nodeHandle);
        void Update();

        /*
         * Control variables
         */
        int aRobots;
        int aId;
        int aQueueSize;
        double aRate;
        double aCommDist;
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
        ros::Publisher aCommunicationModelBroadcaster;
        ros::Publisher aMockCommEvent;

        /*
         * Messages
         */
        std_msgs::Int8MultiArray aRobotsInComm;
        std::vector<geometry_msgs::Pose> aRobotsWorldPoses;

        /*
         * Helpers
         */
        std::vector<tf::Vector3> aRelativePoses;
        std::vector<bool> aReceivedPoses;
};