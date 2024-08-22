/*
 * Copyright (c) 2023, Alysson Ribeiro da Silva
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

#include "RendezvousPlan.h"
#include "ros/ros.h"
#include <string.h>

RendezvousPlan::RendezvousPlan(ros::NodeHandle& nodeHandle, const int& id) {
    this->id = id;

    std::vector<double> K;
    std::vector<double> W;
    int width, height;

    nodeHandle.getParam("/width", width);
    nodeHandle.getParam("/K", K);
    nodeHandle.getParam("/W", W);

    height = static_cast<int>(K.size() / width);
    this->width = width;

    // as a preference, agreements are being stored as rows instead
    // of a single memory block
    for(int y = 0; y < height; ++y) {
        std::vector<agreement_el> row;

        if(K[width * y + id] == 1) {
            for(int x = 0; x < width; ++x) {
                int index = width * y + x;

                agreement_el el;
                el.participate =  K[index];
                el.timer = W[index];

                row.push_back(el);
            }
            this->agreements.push_back(std::pair<int, std::vector<agreement_el>>(y, row));
        }
    }

    current_agreement = 0;
    current_timer = GetCurrentAgreementTimer();
    currentPlanRealization.assign(width,0);
}

RendezvousPlan::~RendezvousPlan() {

}

void RendezvousPlan::PrintRealization() {
    printf("[RendezvousPlan] Realization: ");
    for(size_t robot=0; robot<currentPlanRealization.size(); ++robot) {
        printf("%d ",currentPlanRealization[robot]);
    }
    printf("\n");
}

bool RendezvousPlan::WasPlanRealized() {
    for(size_t robot=0; robot<currentPlanRealization.size(); ++robot) {
        if(currentPlanRealization[robot] == 0 && agreements[current_agreement].second[robot].participate == 1) 
            return false;
    }
    return true;
}

void RendezvousPlan::ResetPlanRealization() {
    current_timer = GetCurrentAgreementTimer();
    currentPlanRealization.assign(this->width, 0);
}

std::vector<int>* RendezvousPlan::GetPlanPtr() {
    return &currentPlanRealization;
}

void RendezvousPlan::RealizePlan(const int& robotId) {
    if(robotId < 0 || robotId >= currentPlanRealization.size())
        throw std::out_of_range("robot id out of range in RealizePlan.");
    currentPlanRealization[robotId] = 1;
}

void RendezvousPlan::Print() {
    std::string to_print = "Rendezvous plan\n\t";
    for(size_t row = 0; row < agreements.size(); ++row) {
        for(size_t col = 0; col < agreements[row].second.size(); ++col) {
            to_print += "[" + std::to_string(agreements[row].second[col].participate) + "," + std::to_string(agreements[row].second[col].timer) + "]";        
        }
        to_print += "\n\t";
    }
    ROS_INFO("[RendezvousPlan] %s", to_print.c_str());
}

void RendezvousPlan::PrintCurrent() {
    int unique_id = GetCurrentAgreementUniqueID();
    ROS_INFO("[RendezvousPlan] id: %d key: %s - timer: %f/%f", 
        unique_id, GenerateAgreementKey(current_agreement).c_str(), current_timer, agreements[current_agreement].second[id].timer);
}

void RendezvousPlan::PrintLocations() {
    std::map<std::string, tf::Vector3>::iterator it = agreements_locations.begin();
    for(;it != agreements_locations.end(); ++it) {
        ROS_INFO("Key %s x: %f y: %f z: %f", it->first.c_str(), it->second.getX(), it->second.getY(), it->second.getZ());
    }
}

int RendezvousPlan::GetCurrentAgreement() {
    return current_agreement;
}

int RendezvousPlan::GetCurrentAgreementConsensusID() {
    for(size_t i = 0; i < agreements[current_agreement].second.size(); ++i) {
        int el = agreements[current_agreement].second[i].participate;
        if(el == 1) return i;
    }
    return -1;
}

bool RendezvousPlan::CheckConsensusCurrentPlan() {
    // this should be pre-computed as some of the other stuff
    int consensus = 0;
    for(size_t i = 0; i < agreements[current_agreement].second.size(); ++i) {
        int el = agreements[current_agreement].second[i].participate;
        if(el == 1) {
            consensus = i;
            break;
        }
    }
    printf("[RendezvousPlan] Consensus id: %d my id: %d\n", consensus, id);
    return (id == consensus);
}

double RendezvousPlan::GetCurrentAgreementTimer() {
    return agreements[current_agreement].second[this->id].timer;
}

void RendezvousPlan::SetNextAgreement() {
    current_agreement += 1;
    if(current_agreement >= agreements.size())
        current_agreement = 0;
    current_timer = GetCurrentAgreementTimer();
}

std::string RendezvousPlan::GenerateAgreementKey(const int& index) {
    if(index < 0 || index >= agreements.size()) throw std::out_of_range("Index out of range in GenerateAgreementKey.");
    std::string key = "";
    for(auto& val : agreements[index].second) {
        key += std::to_string(val.participate);
    }
    return key;
}

void RendezvousPlan::InitializeLocation(tf::Vector3 location) {
    std::string key = "";
    for(size_t agreement = 0; agreement < agreements.size(); ++agreement) {
        key = GenerateAgreementKey(agreement);
        agreements_locations[key] = location;
    }
}

void RendezvousPlan::UpdateCurrentAgreementLocation(tf::Vector3 newLocation) {
    if(current_agreement < 0 || current_agreement >= agreements.size()) throw std::out_of_range("Index out of range in UpdateAgreementLocation.");
    std::string key = GenerateAgreementKey(current_agreement);
    agreements_locations[key] = newLocation;
}

tf::Vector3 RendezvousPlan::GetCurrentAgreementLocation() {
    if(current_agreement < 0 || current_agreement >= agreements.size()) throw std::out_of_range("Index out of range in GetCurrentAgreementLocation.");
    std::string key = GenerateAgreementKey(current_agreement);
    if(agreements_locations.find(key)!=agreements_locations.end()) {
        return agreements_locations[key];
    }
    throw std::runtime_error("Cannot locate key in GetCurrentAgreementLocation");
}

bool RendezvousPlan::ShouldFulfillAgreement() {
    if(current_timer <= 0.0) return true;
    return false;
}

void RendezvousPlan::Update(const double& deltaTime) {
    current_timer -= deltaTime;
}

int RendezvousPlan::GetCurrentAgreementUniqueID() {
    if(current_agreement < 0 || current_agreement >= agreements.size()) throw std::out_of_range("Index out of range in GetCurrentAgreementUniqueID.");
    return agreements[current_agreement].first;
}