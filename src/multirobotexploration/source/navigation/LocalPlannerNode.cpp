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

#include "LocalPlannerNode.h"

LocalPlannerNode::LocalPlannerNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots.");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    if(!node_handle.getParam("controls_to_share", aControlsToShare)) aControlsToShare = 10;
    if(!node_handle.getParam("waypoints_to_use", aMaxWaypoints)) aMaxWaypoints = 30;
    if(!node_handle.getParam("via_points_increment", aViaIncrement)) aViaIncrement = 3;
    if(!node_handle.getParam("use_priority_stop_behavior", aUsePriorityBehavior)) aUsePriorityBehavior = false;
    aNamespace = ros::this_node::getNamespace();

    aReceivedComm = false;
    aSeq = 0;

    // initialize communication containers
    aTebConfig.loadRosParamFromNodeHandle(node_handle);
    aRobotFootprint = teb_local_planner::RobotFootprintModelPtr(new teb_local_planner::PointRobotFootprint());
    aVisual         = teb_local_planner::TebVisualizationPtr(new teb_local_planner::TebVisualization(node_handle, aTebConfig));
    aPlanner        = std::make_shared<teb_local_planner::HomotopyClassPlanner>(aTebConfig, &aObstacleArray, aRobotFootprint, aVisual, &aViaPoints);

    // Subscribers
    aSubscribers.push_back(
        node_handle.subscribe<std_msgs::Int8MultiArray>(
            aNamespace + "/mock_communication_model/robots_in_comm", 
            aQueueSize,
            std::bind(&LocalPlannerNode::RobotInCommCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<nav_msgs::Path>(
            aNamespace + "/integrated_global_planner/current_path", 
            aQueueSize,
            std::bind(&LocalPlannerNode::SubgoalPathCallback, this, std::placeholders::_1)));

    aSubscribers.push_back(
        node_handle.subscribe<multirobotsimulations::CustomPose>(
            aNamespace + "/gmapping_pose/world_pose", 
            aQueueSize, 
            std::bind(&LocalPlannerNode::PoseCallback, this, std::placeholders::_1)));    

    aSubscribers.push_back(
        node_handle.subscribe<costmap_converter::ObstacleArrayMsg>(
            aNamespace + "/costmap_converter/obstacles",
            aQueueSize,
            std::bind(&LocalPlannerNode::ObstacleArrayCallback, this, std::placeholders::_1)));

    // Advertisers
    aVelocityPublisher  = node_handle.advertise<geometry_msgs::Twist>(aNamespace + "/cmd_vel", aQueueSize);    
    aTebPosesPublisher  = node_handle.advertise<geometry_msgs::PoseArray>(aNamespace + "/local_planner/optimal_poses", aQueueSize);
    aViaPointsPublisher = node_handle.advertise<visualization_msgs::Marker>(aNamespace + "/local_planner/global_via_points", aQueueSize);

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&LocalPlannerNode::Update, this)));
}

LocalPlannerNode::~LocalPlannerNode() {

}

void LocalPlannerNode::ObstacleArrayCallback(costmap_converter::ObstacleArrayConstPtr msg) {
    aObstacleArray.clear();
    for(auto& obs : msg->obstacles) {
        teb_local_planner::PolygonObstacle obstacle;
        for(auto& point : obs.polygon.points)
            obstacle.pushBackVertex(point.x, point.y);
        obstacle.finalizePolygon();
        aObstacleArray.push_back(boost::make_shared<teb_local_planner::PolygonObstacle>(obstacle));
    }    
}

void LocalPlannerNode::PoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    aPose.robot_id = msg->robot_id;
    aPose.pose = msg->pose;
}

void LocalPlannerNode::SubgoalPathCallback(nav_msgs::Path::ConstPtr msg) {
    aCurrentPathMsg.poses.assign(msg->poses.begin(), msg->poses.end());
    aCurrentPathMsg.header = msg->header;
}

void LocalPlannerNode::RobotInCommCallback(std_msgs::Int8MultiArray::ConstPtr msg) {
    if(!aReceivedComm) aReceivedComm = true;
    aRobotsInCommMsg.data.assign(msg->data.begin(), msg->data.end());
}

bool LocalPlannerNode::CheckNearPriority() {
    for(size_t robot = 0; robot < aRobotsInCommMsg.data.size(); ++robot) {
        if(robot == aId) continue;
        if(aRobotsInCommMsg.data[robot] == 1 && robot > aId) return true;
    }
    return false;
}

void LocalPlannerNode::CreateMarker(visualization_msgs::Marker& marker, const char* ns, const int& id, const int& seq) {
    marker.id = id;
    marker.header.frame_id = "robot_" + std::to_string(id) + std::string("/map");
    marker.header.stamp = ros::Time().now();
    marker.ns = ns;
    marker.points.clear();
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::MODIFY;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.a = 1.0;
    marker.color.r = 0.3;
    marker.color.g = 0.3;
    marker.color.b = 1.0;
    marker.lifetime = ros::Duration(60);
}

void LocalPlannerNode::AssembleSparsePath(nav_msgs::Path& currentPath, nav_msgs::Path& filteredPath, const int& viaIncrement, visualization_msgs::Marker& globalPathMaker) {
    filteredPath.poses.clear();
    globalPathMaker.points.clear();

    int size = currentPath.poses.size();
    int increment = viaIncrement;

    // waypoint to seek
    size_t waypoint = 0;

    // add the starting pose to the filtered array
    filteredPath.poses.push_back(currentPath.poses[waypoint]);

    // add the path current waypoint into the markers array
    geometry_msgs::Point p;
    p.x = currentPath.poses[waypoint].pose.position.x;
    p.y = currentPath.poses[waypoint].pose.position.y;
    globalPathMaker.points.push_back(p);

    // check how many waypoint should skip when assembling the filtered path
    if(size > increment) waypoint = increment;

    // iterate until the last waypoint
    while(waypoint < size) {
        filteredPath.poses.push_back(currentPath.poses[waypoint]);
        geometry_msgs::Point p;
        p.x = currentPath.poses[waypoint].pose.position.x;
        p.y = currentPath.poses[waypoint].pose.position.y;
        globalPathMaker.points.push_back(p);

        // verify if the increment should change to contemplate the last pose
        if(waypoint + increment >= size && waypoint != size - 1) {
            waypoint = size-2;
            increment = 1;
        }

        waypoint += increment;
    }
}

void LocalPlannerNode::Update() {
    if(!aReceivedComm) return;

    // aways reset velocity
    aTwistVelMsg.linear.x = 0.0;
    aTwistVelMsg.angular.z = 0.0;

    // not optimal mechanism to avoid traffic
    if(aCurrentPathMsg.poses.size() > 1) {
        /*
         * Global path markers
         */
        CreateMarker(aGlobalPathMsg, aNamespace.c_str(), aId, aSeq);
        AssembleSparsePath(aCurrentPathMsg, aFilteredPathMsg, aViaIncrement, aGlobalPathMsg);

        /*
         * Compute sparse via poses
         */
        aViaPoints.clear();
        int via_point = 0;
        while(aViaPoints.size() < aMaxWaypoints && aViaPoints.size() < aFilteredPathMsg.poses.size()) {
            aViaPoints.push_back(
                Eigen::Vector2d(aFilteredPathMsg.poses[via_point].pose.position.x, aFilteredPathMsg.poses[via_point].pose.position.y));
            via_point++;
        }

        /*
         * Compute final pose
         */
        aPrevPoseMsg = aFilteredPathMsg.poses[aViaPoints.size()-2];
        aLastPoseMsg = aFilteredPathMsg.poses[aViaPoints.size()-1];

        // get the yaw from the first to the last point
        double cur_angle = tf::getYaw(aPose.pose.orientation);
        double end_pose_yaw = atan2(
            aLastPoseMsg.pose.position.y - aPrevPoseMsg.pose.position.y, 
            aLastPoseMsg.pose.position.x - aPrevPoseMsg.pose.position.x);

        // optimize trejectory
        aPlanner->plan(teb_local_planner::PoseSE2(aPose.pose.position.x, aPose.pose.position.y, cur_angle), 
                        teb_local_planner::PoseSE2(aLastPoseMsg.pose.position.x, aLastPoseMsg.pose.position.y, end_pose_yaw));
        aPlanner->getVelocityCommand(aTwistVelMsg.linear.x, aTwistVelMsg.linear.y, aTwistVelMsg.angular.z, 1);

        /*
         * Publishers
         */
        aPlanner->visualize();
        aVisual->publishObstacles(aObstacleArray);
        aVisual->publishViaPoints(aViaPoints);
        aViaPointsPublisher.publish(aGlobalPathMsg);

        // controls sharing for traffic avoidance
        aTebPosesMsg.poses.clear();
        teb_local_planner::TebOptimalPlannerPtr best_teb = aPlanner->bestTeb();
        if(best_teb != nullptr) {
            // check how many controls should share
            int to_share = aControlsToShare;
            if(to_share > best_teb->teb().sizePoses()) to_share = best_teb->teb().sizePoses();

            // add the amount of controls into pose array msg
            for (int control=0; control < to_share; ++control) {
                geometry_msgs::Pose to_publish;
                to_publish.position.x = best_teb->teb().Pose(control).x();
                to_publish.position.y = best_teb->teb().Pose(control).y();
                aTebPosesMsg.poses.push_back(to_publish);
            }

            // broadcast controls
            aTebPosesPublisher.publish(aTebPosesMsg);
        }

        // increase sequency for markers
        aSeq += 1;
    }

    /*
     * Check nearby robots to further mitigate traffic
     * this is a naive approach.
     * 
     * TODO:add average velocity to apply a penalty to the 
     * final speed
     * 
     */
    if(CheckNearPriority() && aUsePriorityBehavior) {
        aTwistVelMsg.linear.x /= 2.0;
        aTwistVelMsg.angular.z /= 2.0;
    }

    // ways send velocity to robot
    aVelocityPublisher.publish(aTwistVelMsg);
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "localplannernode");
    std::unique_ptr<LocalPlannerNode> localPlannerNode = std::make_unique<LocalPlannerNode>();
    ros::spin();
}