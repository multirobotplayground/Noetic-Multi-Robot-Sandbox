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

#include "MockCommunicationModelNode.h"

/*
 * Node implementation
 */
MockCommunicationModelNode::MockCommunicationModelNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("/robots", aRobots)) throw std::runtime_error("Could not retrieve /robots.");
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("comm_dist", aCommDist)) throw std::runtime_error("Could not retrieve comm_dist.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;
    aNamespace = ros::this_node::getNamespace();

    // initialize communication containers
    aReceivedPoses.assign(aRobots, false);
    aRobotsWorldPoses.assign(aRobots, geometry_msgs::Pose());
    aRelativePoses.assign(aRobots, tf::Vector3());
    aRobotsInComm.data.assign(aRobots, 0);
    LoadRelativePoses(node_handle);

    /*
     * I use lambda in the multi-robot context subscriptions, because it can help 
     * having multiple callbacks, one for each robot, without explicitly
     * defining them, which would required some engineering.
     */
    std::vector<bool>* receivedPosesPtr = &aReceivedPoses;
    std::vector<geometry_msgs::Pose>* robotsWorldPosesPtr = &aRobotsWorldPoses;
    for(int robot = 0; robot < aRobots; ++robot) {    
    aSubscribers.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>("/robot_" + std::to_string(robot) + "/gmapping_pose/world_pose", aQueueSize, 
        [robot, receivedPosesPtr, robotsWorldPosesPtr](multirobotsimulations::CustomPose::ConstPtr msg) {
            robotsWorldPosesPtr->at(robot).position = msg->pose.position;
            robotsWorldPosesPtr->at(robot).orientation = msg->pose.orientation;
            receivedPosesPtr->at(robot) = true;
        }));
    }

    // Advertisers
    aCommunicationModelBroadcaster = node_handle.advertise<std_msgs::Int8MultiArray>(aNamespace + "/mock_communication_model/robots_in_comm", aQueueSize);

    // Node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&MockCommunicationModelNode::Update, this)));
}

MockCommunicationModelNode::~MockCommunicationModelNode() {

}

void MockCommunicationModelNode::LoadRelativePoses( ros::NodeHandle& nodeHandle) {
    // read relative start poses parameters
    std::vector<std::map<std::string, double>> poses;
    std::string key = "";
    for (int robot = 0; robot < aRobots; ++robot) {
        key = "/start_pose_robot_" + std::to_string(robot);
        std::map<std::string, double> pose;
        nodeHandle.getParam(key, pose);
        poses.push_back(pose);
        ROS_INFO("[MockCommunicationModelNode]: %s: %f %f %f", key.c_str(), pose["x"], pose["y"], pose["z"]);
    }

    // compute relative poses from file
    tf::Vector3 my_pose(poses[aId]["x"], poses[aId]["y"], poses[aId]["z"]);
    for(int robot = 0; robot < aRobots; ++robot) {
        if(robot!=aId) {
            tf::Vector3 other_pose(poses[robot]["x"], poses[robot]["y"], poses[robot]["z"]);
            tf::Vector3 dir = other_pose - my_pose;
            aRelativePoses[robot] = dir;
            ROS_INFO("[MockCommunicationModelNode] relative to %d: %f %f %f", robot, dir.getX(), dir.getY(), dir.getZ());
        } else {
            aRelativePoses[robot].setX(0.0);
            aRelativePoses[robot].setY(0.0);
            aRelativePoses[robot].setZ(0.0);
            ROS_INFO("[MockCommunicationModelNode] relative to self: %f %f %f", aRelativePoses[robot].getX(), aRelativePoses[robot].getY(), aRelativePoses[robot].getZ());
        }
    }
}

void MockCommunicationModelNode::Update() {
    // translate all the poses here
    tf::Vector3 relative_pose;
    tf::Vector3 my_pose;
    PoseToVector3(aRobotsWorldPoses[aId], my_pose);

    double distance = 0.0;
    for(int robot = 0; robot < aRobots; ++robot) {
        if(aReceivedPoses[robot] == true) {
            PoseToVector3(aRobotsWorldPoses[robot], relative_pose);
            relative_pose += aRelativePoses[robot];

            // check distance to me to see if this the relative positions should be updated
            distance = relative_pose.distance(my_pose);

            if(distance < aCommDist) {
                // set nearby robots
                aRobotsInComm.data[robot] = 1;
            } else {
                aRobotsInComm.data[robot] = 0;
            }
        }
    }

    // send data to the system
    aCommunicationModelBroadcaster.publish(aRobotsInComm);
}

/*
 * Node's main function
 */
int main(int argc, char* argv[]) {
    ros::init(argc, argv, "mockcommunicationmodelnode");
    std::unique_ptr<MockCommunicationModelNode> mockCommunicationModelNode = std::make_unique<MockCommunicationModelNode>();
    ros::spin();
}