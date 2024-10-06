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

struct agreement_el {
    int participate;
    double timer;
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
        void UpdateCurrentAgreementLocation(tf::Vector3 newLocation);
        tf::Vector3 GetCurrentAgreementLocation();
        void PrintLocations();
        void Print();
        void PrintCurrent();
        void PrintRealization();
        int GetCurrentAgreementUniqueID();
        int GetCurrentAgreementConsensusID();

        // relization
        void RealizePlan(const int& robotId);
        void ResetPlanRealization();
        bool WasPlanRealized();

        std::vector<int>* GetPlanPtr();
        
    private:
        std::string GenerateAgreementKey(const int& index);

        int id, width;
        int current_agreement;
        double current_timer;
        std::vector< std::pair<int, std::vector<agreement_el>> > agreements;
        std::map<std::string, tf::Vector3> agreements_locations;

        std::vector<std::vector<agreement_el>> agreementsIParticipate;

        std::vector<int> currentPlanRealization;
};