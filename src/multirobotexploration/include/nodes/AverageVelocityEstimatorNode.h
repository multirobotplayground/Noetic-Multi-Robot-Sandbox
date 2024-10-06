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
#include <deque>
#include <string.h>
#include "ros/ros.h"
#include "tf/tf.h"

/*
 * Messages
 */
#include "std_msgs/Float32.h"
#include "geometry_msgs/Pose.h"
#include "multirobotsimulations/CustomPose.h"

/*
 * Helpers
 */
#include "Common.h"

/*
 * AverageVelocityEstimatorNode class
 */
class AverageVelocityEstimatorNode {
    public:
        AverageVelocityEstimatorNode();
        ~AverageVelocityEstimatorNode();

    private:
        double ComputeAverageVelocity(std::deque<double>& speedArray);
        void WorldPoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void Update();

        /*
         * Control variables
         */
        int aId;
        int aQueueSize;
        int aCount;
        bool aReceivedPosition;
        double aRate;
        std::string aNamespace;
        tf::Vector3 aLastWorldPos;
        tf::Vector3 aWorldPos;

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
        ros::Publisher aAverageVelocityPublisher;

        /*
         * Messages
         */
        std_msgs::Float32 aAverageVelocityMsg;

        /*
         * Helpers
         */
         std::deque<double> aVelocityArray;
};
