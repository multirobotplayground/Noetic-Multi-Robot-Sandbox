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