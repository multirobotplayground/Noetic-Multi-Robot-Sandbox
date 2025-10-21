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

#include <vector>
#include "tf/tf.h"
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

struct agreement_el {
    int participate;
    double timer;
};

class Stats {
    public:
        Stats(int id, double time_waiting, int rendezvous_id, double total_time, double time_to_reach_next_rendezvous) {
            this->id = id;
            this->time_waiting = time_waiting;
            this->rendezvous_id = rendezvous_id;
            this->total_time = total_time;
            this->time_to_reach_next_rendezvous = time_to_reach_next_rendezvous;
        }
        ~Stats() {

        }

        int id;
        double time_waiting;
        int rendezvous_id;
        double total_time; // Total time spent in the mission
        double time_to_reach_next_rendezvous; // Time to reach the next rendezvous
};

class StatsArr {
    public:
        StatsArr() {
            stats.clear();
        }
        ~StatsArr() {
            stats.clear();
        }

        void Add(Stats s) {
            stats.push_back(s);
        }

        void Send(ros::Publisher& pub) {
            
            for(const auto& s : stats) {
                std_msgs::Float64MultiArray msg;
                msg.data.push_back(s.id);
                msg.data.push_back(s.time_waiting);
                msg.data.push_back(s.rendezvous_id);
                msg.data.push_back(s.total_time);
                msg.data.push_back(s.time_to_reach_next_rendezvous);
                pub.publish(msg);
                sleep(0.1); // Sleep to allow the message to be sent
            }
        }

        void Print() {
            ROS_INFO("################################################");
            ROS_INFO("###### Waiting times and rendezvous stats ######");
            ROS_INFO("################################################");
            for(const auto& s : stats) {
                ROS_INFO("[Alysson2024Node] Robot %d: Time waiting: %.2f, Rendezvous ID: %d, Total time: %.2f, Time to next rendezvous: %.2f",
                         s.id, s.time_waiting, s.rendezvous_id, s.total_time, s.time_to_reach_next_rendezvous);
            }
        }
    private:
        std::vector<Stats> stats; // Vector to hold stats for each robot
};

class RendezvousPlan {
    public:
        RendezvousPlan(ros::NodeHandle& nodeHandle, const int& id);
        ~RendezvousPlan();

        int GetCurrentAgreement();
        double GetCurrentAgreementTimer();
        bool CheckConsensusCurrentPlan();

        void InitializeLocation(tf::Vector3 location);
        void Update(const double& deltaTime);
        bool ShouldFulfillAgreement();
        void SetNextAgreement();
        bool SetNextAgreementNonLinked();
        void UpdateCurrentAgreementLocation(tf::Vector3 newLocation);
        tf::Vector3 GetCurrentAgreementLocation();
        void PrintLocations();
        void Print();
        void PrintCurrent();
        void PrintRealization();
        void SetCurrentTime(const double& time);
        int GetCurrentAgreementUniqueID();
        int GetCurrentAgreementConsensusID();
        bool HasValidAgreement();
        bool PairwiseRule(const int& robotId);

        // relization
        void RealizePlan(const int& robotId);
        void ResetPlanRealization();
        bool WasPlanRealized();

        void UpdatePlan(tf::Vector3 newLocation);
        void SkipPlan();
        void Reset();

        std::vector<int>* GetPlanPtr();
        
        double getCurrentGlobalCompletionTime() {
            if(current_agreement < 0 || current_agreement >= agreements.size()) -1.0;
            return globalCompletionTimes[current_agreement];
        }

    private:
        std::string GenerateAgreementKey(const int& index);

        int id, width;
        int current_agreement;
        double current_timer;
        std::vector< std::pair<int, std::vector<agreement_el>> > agreements;
        std::map<std::string, tf::Vector3> agreements_locations;

        std::vector<std::vector<agreement_el>> agreementsIParticipate;
        std::vector<double> globalCompletionTimes;

        std::vector<int> currentPlanRealization;
};