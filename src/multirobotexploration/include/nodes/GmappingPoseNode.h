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
