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
 * Ros and system
 */
#include <stdio.h>
#include <queue>
#include "ros/ros.h"
#include "tf/tf.h"

/*
 * Messages
 */
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Path.h"
#include "nav_msgs/OccupancyGrid.h"
#include "visualization_msgs/Marker.h"
#include "geometry_msgs/PoseStamped.h"
#include "multirobotsimulations/CustomPose.h"

/*
 * Helpers
 */
#include "SearchAlgorithms.h"
#include "Common.h"

typedef enum {
    state_idle = 0,
    state_executing_path = 1
} SubGoalState;

class IntegratedGlobalPlannerNode {
    public:
        IntegratedGlobalPlannerNode();
        ~IntegratedGlobalPlannerNode();

    private:
        void CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq);
        void DepthFirstSearchFreePath(
            nav_msgs::OccupancyGrid& cspace, 
            Vec2i& occpos,
            Vec2i& source, 
            Vec2i& closest,
            std::list<Vec2i>& outpath);
        void WavefrontPath(nav_msgs::OccupancyGrid& cspace, 
                           Vec2i& occpos,
                           Vec2i& target, 
                           Vec2i& closest,
                           std::list<Vec2i>& outpath);
        void ChangeState(const SubGoalState& newState);
        void CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        void PoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void AverageVelocityCallback(std_msgs::Float32::ConstPtr msg);
        void GoalCallback(geometry_msgs::Pose::ConstPtr msg);
        void StopCallBack(std_msgs::String::ConstPtr msg);
        void Update();

        /*
         * Control variables
         */
        int aQueueSize;
        int aId;
        int aSeq;
        int aDeltaTimeSec;
        bool aHasPose;
        bool aHasOcc;
        bool aHasAverageVelocity;
        double aRate;
        double aDistance;
        double aSubGoalReachThreshold;
        double aStuckTimeThreshold;
        double aStuckTime;
        double aAverageVelocity;
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
        ros::Publisher aPathMarkerPublisher;
        ros::Publisher aFinishEventPublisher;
        ros::Publisher aCurrentPathPublisher;

        /*
         * Messages
         */
        nav_msgs::Path aPathMsg;
        std_msgs::String aStrMsg;
        visualization_msgs::Marker aPathMarkerMsg;
        nav_msgs::OccupancyGrid aCspace;

        /*
         * Helpers
         */
        ros::Time last_time;
        std::list<Vec2i> aWaypoints;
        tf::Vector3 aLastPos;
        tf::Vector3 aWorldPos;
        tf::Vector3 aCurrentGoal;
        Vec2i aOccPos;
        SubGoalState aCurrentState;
};