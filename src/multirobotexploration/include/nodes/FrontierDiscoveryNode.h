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
#include "nav_msgs/OccupancyGrid.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseArray.h"
#include "std_msgs/String.h"
#include "multirobotsimulations/CustomPose.h"
#include "multirobotsimulations/Frontiers.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "multirobotsimulations/frontierservice.h"

/*
 * Helpers
 */
#include "Common.h"
#include "SearchAlgorithms.h"

/*
 * FrontierDiscoveryNode states
 */
typedef enum{
    IDLE = 0,
    PROCESSING = 1,
    FINISHED = 2
}FrontierState;

/*
 * Frontier discovery node class
 */
class FrontierDiscoveryNode {
    public:
        FrontierDiscoveryNode();
        ~FrontierDiscoveryNode();

    private:
        void Update();
        void CSpaceCallback(nav_msgs::OccupancyGrid::ConstPtr msg);
        void EstimatePoseCallback(multirobotsimulations::CustomPose::ConstPtr msg);
        void ComputeCallback(std_msgs::String::ConstPtr msg);
        void CreateMarker(visualization_msgs::Marker& input, const char* ns, const int& id, const int& seq);
        void SetPoseArr(geometry_msgs::PoseArray& arr, const int& seq);
        void ResetFrontierMsg(multirobotsimulations::Frontiers& msg);
        void CheckReachableFrontiers(nav_msgs::OccupancyGrid& occ, nav_msgs::OccupancyGrid& out, const Vec2i& pos, std::vector<Vec2i>& reachable_frontiers);
        double ComputeCentroidValue(nav_msgs::OccupancyGrid& occ, Vec2i& centroid, const double& lidarRange);
        bool ServiceRequest(multirobotsimulations::frontierservice::Request& req, multirobotsimulations::frontierservice::Response& res);

        /*
         * Control variables
         */
        int aQueueSize;
        int aId;
        int aSeq;
        int aClusterDetectionMin;
        bool aReceivedCSpace;
        bool aHasPose;
        double aRate;
        double aYaw;
        double aMaxLidarRange;
        Vec2i aPos;
        FrontierState aState;
        std::string aNamespace;
        ros::ServiceServer aFrontierRequestService;

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
        ros::Publisher aClusterMarkerPub;
        ros::Publisher aFrontiersMapPub;
        ros::Publisher aFrontiersClustersPub;

        /*
         * Messages
         */
        multirobotsimulations::Frontiers aFrontierMsg;
        geometry_msgs::PoseArray aPoseArrMsg;
        geometry_msgs::Pose aWorldPos;
        nav_msgs::OccupancyGrid aOcc;
        nav_msgs::OccupancyGrid aFrontiersMap;
        visualization_msgs::Marker aClusterMarkerMsg;

        /*
         * Helpers
         */
        std::list<Vec2i> aPath;
        std::vector<Vec2i> aFrontiers;
        std::vector<Vec2i> aCentroids;
        std::vector<Vec2i> aFilteredCentroids;
        std::vector<std::vector<Vec2i>> aClusters;
        std::vector<std::vector<Vec2i>> aFilteredClusters;
};
