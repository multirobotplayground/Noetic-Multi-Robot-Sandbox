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
            double max = std::numeric_limits<double>::min();
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

void RendezvousPlan::Reset() {
    current_agreement = 0;
    current_timer = GetCurrentAgreementTimer();
    currentPlanRealization.assign(this->width, 0);
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
    int index = current_agreement;
    if(current_agreement >= agreements.size()) index = agreements.size() - 1;
    ROS_INFO("[RendezvousPlan] id: %d key: %s - timer: %f/%f", 
        unique_id, GenerateAgreementKey(index).c_str(), current_timer, agreements[index].second[id].timer);
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
    /*
    * IF ALL WAS ACOMPLISHED, SET THE LOCATION OF THE LAST AGREEMENT
    */
    int index = current_agreement;
    if(current_agreement >= agreements.size()) index = agreements.size() - 1;

    for(size_t i = 0; i < agreements[index].second.size(); ++i) {
        int el = agreements[index].second[i].participate;
        if(el == 1) return i;
    }
    return -1;
}

bool RendezvousPlan::CheckConsensusCurrentPlan() {
    /*
    * IF ALL WAS ACOMPLISHED, SET THE LOCATION OF THE LAST AGREEMENT
    */
    int cur = current_agreement;
    if(!HasValidAgreement())
        cur = agreements.size() - 1;

    int consensus = 0;
    for(size_t i = 0; i < agreements[current_agreement].second.size(); ++i) {
        int el = agreements[current_agreement].second[i].participate;
        if(el == 1) {
            consensus = i;
            break;
        }
    }
    return (id == consensus);
}

void RendezvousPlan::SetCurrentTime(const double& time) {
    current_timer = time;
}

double RendezvousPlan::GetCurrentAgreementTimer() {
    /*
    * IF ALL WAS ACOMPLISHED, SET THE LOCATION OF THE LAST AGREEMENT
    */

    int index = current_agreement;
    if(current_agreement >= agreements.size()) index = agreements.size() - 1;
    return agreements[index].second[this->id].timer;
}

void RendezvousPlan::SetNextAgreement() {
    current_agreement += 1;
    if(current_agreement >= agreements.size())
        current_agreement = 0;
    current_timer = GetCurrentAgreementTimer();
}

bool RendezvousPlan::HasValidAgreement() {
    return (current_agreement >= 0 && current_agreement < agreements.size());
}

bool RendezvousPlan::SetNextAgreementNonLinked() {
    current_agreement += 1;
    bool status = true;
    if(current_agreement >= agreements.size())
        status = false;

    current_timer = GetCurrentAgreementTimer();
    return status;
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
    if(current_agreement < 0) throw std::out_of_range("Index out of range in UpdateAgreementLocation.");
    int index = current_agreement;

    /*
    * IF ALL WAS ACOMPLISHED, SET THE LOCATION OF THE LAST AGREEMENT
    */
    if(current_agreement >= agreements.size()) index = current_agreement -1;
    std::string key = GenerateAgreementKey(index);
    agreements_locations[key] = newLocation;
}

tf::Vector3 RendezvousPlan::GetCurrentAgreementLocation() {
     if(current_agreement < 0) throw std::out_of_range("Index out of range in GetCurrentAgreementLocation.");
     std::string key = "";

    /*
    * IF ALL WAS ACOMPLISHED, RETURN THE LAST AGREEMENT
    */
    if(current_agreement >= agreements.size())
        key = GenerateAgreementKey(agreements.size() - 1);
    else
        key = GenerateAgreementKey(current_agreement);
    if(agreements_locations.find(key)!=agreements_locations.end()) {
        return agreements_locations[key];
    }
    throw std::runtime_error("Cannot locate key in GetCurrentAgreementLocation");
}

bool RendezvousPlan::ShouldFulfillAgreement() {
    /*
    * NEVER REALIZE IF ALL WAS ACOMPLISHED
    */
    if(current_agreement < 0 || current_agreement >= agreements.size()) return false;
    if(current_timer <= 0.0) return true;
    return false;
}

void RendezvousPlan::Update(const double& deltaTime) {
    if(current_agreement < 0 || current_agreement >= agreements.size()) return;
    current_timer -= deltaTime;
}

int RendezvousPlan::GetCurrentAgreementUniqueID() {
    /*
    * RETURN LAST AGREEMENT ID IF FINISHES THE PLAN
    */
    if(current_agreement < 0) return -1;
    if(current_agreement >= agreements.size()) {
        return agreements[current_agreement-1].first;
    }
    return agreements[current_agreement].first;
}

void RendezvousPlan::UpdatePlan(tf::Vector3 newLocation) {
    if(current_agreement >= 0 && current_agreement < agreements.size()) {
        UpdateCurrentAgreementLocation(newLocation);
        ResetPlanRealization();
        SetNextAgreementNonLinked();
    }
}

void RendezvousPlan::SkipPlan() {
    if(current_agreement >= 0 && current_agreement < agreements.size())
        SetNextAgreementNonLinked();
    ResetPlanRealization();
}

bool RendezvousPlan::PairwiseRule(const int& robotId) {
    if(robotId < 0 || robotId >= currentPlanRealization.size())
        throw std::out_of_range("robot id out of range in PairwiseRule.");
  
    bool other_participate = false;
    int num_robots_participating = 0;
    for(size_t i = 0; i < agreements[current_agreement].second.size(); ++i) {
        if(agreements[current_agreement].second[i].participate == 1) {
            num_robots_participating++;
        }
    }
    other_participate = agreements[current_agreement].second[robotId].participate == 1;
    
    if(num_robots_participating == 2 && other_participate) return true;
    return false;
}