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
 * CSpaceNode class
 */
class CSpaceNode {
    public:
        CSpaceNode();
        ~CSpaceNode();

    private:
        void Inflate(nav_msgs::OccupancyGrid& occ,
                        const double& freeInflationRadius,
                        const double& occupiedInflationRadius, 
                        const int8_t& occupancyThreshold = 90,
                        const int8_t& freeThreshold = 50,
                        const int8_t& occupiedValue = 100,
                        const int8_t& freeVal = 1);

        void OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        void WorldPoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void Update();
        void CommunicationsCallback(std_msgs::Int8MultiArray::ConstPtr msg);

        /*
         * Control variables
         */
        int aQueueSize;
        int aId;
        bool aHasOcc;
        int aRobots;
        bool aMarkRobotsForLocalPlanning;
        bool aHasComm;
        double aRate;
        double aFreeInflateRadius;
        double aOccuInflateRadius;
        tf::Vector3 aOccPose;
        std::string aNamespace;

        /*
         * Routines
         */
        std::vector<ros::Timer> aTimers;
        std::vector<geometry_msgs::Pose> aWorldPoses;
        std::vector<bool> aReceivedPoses;

        /*
         * Subscribers
         */
        std::vector<ros::Subscriber> aSubscribers;
        
        /*
         * Advertisers
         */   
        ros::Publisher aCspacePublisher;

        /*
         * Messages
         */
        nav_msgs::OccupancyGrid aOccMsg;
        nav_msgs::OccupancyGrid aCspaceMsg;
        std_msgs::Int8MultiArray aRobotsInCommMsg;
};
