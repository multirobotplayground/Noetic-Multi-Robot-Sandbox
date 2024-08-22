/*
 * Copyright (c) 2020, Alysson Ribeiro da Silva
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. All advertising materials mentioning features or use of this software must
 *    display the following acknowledgement:
 *    This product includes software developed by Alysson Ribeiro da Silva.
 * 
 * 4. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * 5. The source and the binary form, and any modifications made to them
 *    may not be used for the purpose of training or improving machine learning
 *    algorithms, including but not limited to artificial intelligence, natural
 *    language processing, or data mining. This condition applies to any derivatives,
 *    modifications, or updates based on the Software code. Any usage of the source
 *    or the binary form in an AI-training dataset is considered a breach of
 *    this License.
 * 
 * THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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
        double ComputeCentroidValue(nav_msgs::OccupancyGrid& occ, Vec2i& centroid, const double& lidarRange);

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
