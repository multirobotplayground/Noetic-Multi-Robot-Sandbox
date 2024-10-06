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
#include "ros/ros.h"
#include "teb_local_planner/teb_local_planner_ros.h"

/*
 * Messages
 */
#include "std_msgs/Int8MultiArray.h"
#include "std_msgs/Float32.h"
#include "multirobotsimulations/CustomPose.h"
#include "multirobotsimulations/MockPackage.h"
#include "std_msgs/Float64MultiArray.h"
#include "geometry_msgs/PoseArray.h"

/*
 *
 */
#include "Common.h"

class LocalPlannerNode {
    public:
        LocalPlannerNode();
        ~LocalPlannerNode();

    private:
        void CreateMarker(visualization_msgs::Marker& marker, const char* ns, const int& id, const int& seq);
        void AssembleSparsePath(nav_msgs::Path& currentPath, nav_msgs::Path& filteredPath, const int& viaIncrement, visualization_msgs::Marker& globalPathMaker);
        bool CheckNearPriority();
        void ObstacleArrayCallback(costmap_converter::ObstacleArrayConstPtr msg);
        void PoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void SubgoalPathCallback(nav_msgs::Path::ConstPtr msg);
        void RobotInCommCallback(std_msgs::Int8MultiArray::ConstPtr msg);
        void Update();

        /*
         * Control variables
         */
        int aQueueSize;
        int aSeq;
        int aId;
        int aRobots;
        int aMaxWaypoints;
        int aViaIncrement;
        int aIncrement;
        int aControlsToShare;
        bool aUsePriorityBehavior;
        bool aReceivedComm;
        double aRate;
        std::string aNamespace;
        std::string aName;

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
        ros::Publisher aVelocityPublisher;
        ros::Publisher aTebPosesPublisher;
        ros::Publisher aViaPointsPublisher;
        
        /*
         * Messages
         */
        geometry_msgs::Twist aTwistVelMsg;
        geometry_msgs::PoseStamped aPrevPoseMsg;
        geometry_msgs::PoseStamped aLastPoseMsg;
        geometry_msgs::PoseArray aTebPosesMsg;
        visualization_msgs::Marker aGlobalPathMsg;
        std_msgs::Int8MultiArray aRobotsInCommMsg;
        nav_msgs::Path aCurrentPathMsg;
        nav_msgs::Path aFilteredPathMsg;
        multirobotsimulations::CustomPose aPose;

        /*
         * Helpers
         */
        teb_local_planner::TebConfig aTebConfig;
        teb_local_planner::ViaPointContainer aViaPoints;
        teb_local_planner::TebVisualizationPtr aVisual;
        teb_local_planner::RobotFootprintModelPtr aRobotFootprint;
        std::vector<teb_local_planner::ObstaclePtr> aObstacleArray;
        std::shared_ptr<teb_local_planner::HomotopyClassPlanner> aPlanner;
};