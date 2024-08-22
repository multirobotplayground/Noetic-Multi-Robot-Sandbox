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

#include "AverageVelocityEstimatorNode.h"

AverageVelocityEstimatorNode::AverageVelocityEstimatorNode() {
    ros::NodeHandle node_handle("~");

    // load all parameters
    if(!node_handle.getParam("id", aId)) throw std::runtime_error("Could not retrieve id.");
    if(!node_handle.getParam("rate", aRate)) aRate = 2.0;;
    if(!node_handle.getParam("queue_size", aQueueSize)) aQueueSize = 2;
    if(!node_handle.getParam("count", aCount)) aCount = 10;
    aNamespace = ros::this_node::getNamespace();

    // subscriptions
    aSubscribers.push_back(node_handle.subscribe<multirobotsimulations::CustomPose>(
        aNamespace + "/gmapping_pose/world_pose", 
        aQueueSize, 
        std::bind(&AverageVelocityEstimatorNode::WorldPoseCallback, this, std::placeholders::_1)));

    // advertisers
    aAverageVelocityPublisher = node_handle.advertise<std_msgs::Float32>(aNamespace + "/average_velocity", aQueueSize);

    // node's routines
    double update_period = PeriodToFreqAndFreqToPeriod(aRate);
    aTimers.push_back(node_handle.createTimer(ros::Duration(update_period), std::bind(&AverageVelocityEstimatorNode::Update, this)));   
}

AverageVelocityEstimatorNode::~AverageVelocityEstimatorNode() {
    
}

void AverageVelocityEstimatorNode::WorldPoseCallback(multirobotsimulations::CustomPose::ConstPtr msg) {
    aWorldPos.setX(msg->pose.position.x);
    aWorldPos.setY(msg->pose.position.y);

    // initialize last pos, once as soon
    // as a pose is received
    if(!aReceivedPosition) { 
        aLastWorldPos = aWorldPos;
        aReceivedPosition = true;
    }
}

double AverageVelocityEstimatorNode::ComputeAverageVelocity(std::deque<double>& speedArray) {
    double average = 0.0;
    double velocities = static_cast<double>(aVelocityArray.size());
    
    if(velocities > 0) {
        for(std::deque<double>::iterator i = aVelocityArray.begin(); i != aVelocityArray.end(); ++i) average += (*i);
        average /= velocities;
    }

    return average;   
}

void AverageVelocityEstimatorNode::Update() {
    // compute moving average
    if(aReceivedPosition) {
        aAverageVelocityMsg.data = ComputeAverageVelocity(aVelocityArray);
        aAverageVelocityPublisher.publish(aAverageVelocityMsg);
    }

    // update velocities array
    aVelocityArray.push_back(aWorldPos.distance(aLastWorldPos));
    if(aVelocityArray.size() > aCount) aVelocityArray.pop_front();
    aLastWorldPos = aWorldPos;    
}

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "averagespeedestimatornode");
    std::unique_ptr<AverageVelocityEstimatorNode> averagePoseEstimatorNode = std::make_unique<AverageVelocityEstimatorNode>();
    ros::spin();
}