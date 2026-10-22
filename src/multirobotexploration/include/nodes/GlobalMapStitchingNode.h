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
 * Ros and system
 */
#include <string.h>
#include <iostream>
#include <signal.h>
#include <vector>
#include "ros/ros.h"
#include "ros/package.h"

/*
 * Messages
 */
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/PoseArray.h"
#include "multirobotsimulations/CustomPose.h"
#include "multirobotsimulations/MockPackage.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Int8MultiArray.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Bool.h"

/*
 * Helpers
 */
#include "Common.h"

class GlobalMapStitchingNode {
    public:
        GlobalMapStitchingNode();
        ~GlobalMapStitchingNode();

    private:
        void CommunicationsCallback(std_msgs::Int8MultiArray::ConstPtr msg);
        void RelativeStartingPosesCallback(geometry_msgs::PoseArray::ConstPtr msg);
        void OccCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        nav_msgs::OccupancyGrid Stitch(nav_msgs::OccupancyGrid& A, nav_msgs::OccupancyGrid& B);
        void set_value(nav_msgs::OccupancyGrid& grid, const int& x, const int& y, int8_t value);
        void Update();
        bool ResetCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);

        /*
         * Control variables
         */
        int aQueueSize;
        int aId;
        int aRobots;
        bool aDirty;
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
        ros::Publisher aFusionPublisher;
        
        /*
         * Services
         */
        ros::ServiceServer aResetService;

        /*
         * Messages
         */
        nav_msgs::OccupancyGrid aFusionMsg;
        std_msgs::Int8MultiArray aRobotsInCommMsg;
        geometry_msgs::PoseArray aRobotsRelativePosesMsg;

        /*
         * Helpers
         */
        std::vector<nav_msgs::OccupancyGrid> aRobotsOcc;  
        std::vector<bool> aReceivedOccs;
        bool aReceivedRelativePoses;
        bool aReset;
        bool aGlobalPause;

        void globalPauseCallback(std_msgs::Bool::ConstPtr msg);
};