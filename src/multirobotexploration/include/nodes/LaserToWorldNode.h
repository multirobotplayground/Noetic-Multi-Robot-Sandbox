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
#include <vector>
#include "ros/ros.h"
#include "tf/tf.h"

/*
 * Messages
 */
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseArray.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "multirobotsimulations/CustomPose.h"

/*
 * Helpers
 */
#include "Common.h"

class LaserToWorldNode {
    public:
        LaserToWorldNode();
        ~LaserToWorldNode();

    private:
        void EstimatePoseWorldCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void OccupancyGridCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        void LaserCapture(sensor_msgs::LaserScan::ConstPtr msg);
        void Update();

        /*
         * Control variables
         */
        int aQueueSize;
        bool aHasLidar;
        bool aHasPose;
        bool aHasOccInfo;
        double aRate;
        double aRobotYaw;
        double aLidarError;
        tf::Vector3 aRobotWorldPosition;
        tf::Vector3 aLidarPosition;
        tf::Quaternion aLidarOrientation;
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
        ros::Publisher aLidarPublisher;
        ros::Publisher aOccLidarPublisher;

        /*
         * Messages
         */
        geometry_msgs::PoseArray aWorldLidarMsg;
        geometry_msgs::PoseArray aOccLidarMsg;
        nav_msgs::OccupancyGrid aOccInfo;

        /*
         * Helpers
         */
        std::vector<geometry_msgs::Pose> aWorldReadings;
        std::vector<geometry_msgs::Pose> aOccReadings;
};
