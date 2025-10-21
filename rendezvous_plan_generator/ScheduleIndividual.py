# Job-shop Scheduling for Multi-robot Intermittent Communication Generator
# Copyright (C) 2025 Alysson Ribeiro da Silva
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import numpy as np
import os
import datetime

class schedule_struct:
    def __init__(self, decisions, weights):
        self.decisions = decisions
        self.weights = weights

#
#
# [decision variables ...] [weight variables or scalars]
#
class ScheduleIndividual:
    def __init__(self, robots, schedules):
        self.robots = robots
        self.schedules = schedules

        self.total_el = robots * schedules
        self.decision_variables = np.zeros(self.total_el, dtype=float)
        self.weights = np.zeros(self.total_el, dtype=float)

    def set_solution_mips(self, decisions, jobs_s, jobs_e):
        for i in range(self.schedules):
            for j in range(self.robots):
                self.set_decision(i,j, decisions[i][j].x)
                self.set_weight(i,j, jobs_e[i][j].x - jobs_s[i][j].x)

    def set_decision(self, i, j, value):
        index = i * self.robots + j
        self.decision_variables[index] = value

    def set_weight(self, i, j, value):
        index = i * self.robots + j
        self.weights[index] = value

    def set_solution(self, solution):
        self.decision_variables = solution[:self.total_el]
        self.weights = solution[self.total_el:self.total_el*2]

    def print_individual(self):
        print("individual:\n")
        for schedule in range(self.schedules):
            for robot in range(self.robots):
                decision, weight = self.get_el(robot, schedule)
                print("[{} {}]".format(decision, weight), end=" ")
            print()

    def save_file(self, path, filename="schedule"):
        complete_path = os.path.join(path, filename)
        with open(complete_path, 'w') as f:
            f.write("# Noetic-Multi-Robot-Sandbox for multi-robot research using ROS Noetic\n")
            f.write(f"# Copyright (C) {datetime.date.today().year} Alysson Ribeiro da Silva\n")
            f.write("# \n")
            f.write("# This program is free software: you can redistribute it and/or modify\n")
            f.write("# it under the terms of the GNU General Public License as published by\n")
            f.write("# the Free Software Foundation, either version 3 of the License, or\n")
            f.write("# (at your option) any later version.\n")
            f.write("#\n")
            f.write("# This program is distributed in the hope that it will be useful,\n")
            f.write("# but WITHOUT ANY WARRANTY; without even the implied warranty of\n")
            f.write("# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the\n")
            f.write("# GNU General Public License for more details.\n")
            f.write("#\n")
            f.write("# You should have received a copy of the GNU General Public License\n")
            f.write("# along with this program.  If not, see <http://www.gnu.org/licenses/>.\n")
            f.write("#\n")
            f.write("# Width of the plan is used to get the number of agreements\n")
            f.write("# and iterate over the matrix.\n")

            f.write("width: {}\n\n".format(self.robots))

            f.write("# The k matrix represents the rendezvous plan, where each row is an agreement\n")
            f.write("# and columns represent robots.\n")
            f.write("K: [")
            for i in range(len(self.decision_variables)):  
                if i == len(self.decision_variables) - 1:
                    f.write("{}]".format(self.decision_variables[i]))
                else:
                    f.write("{}, ".format(self.decision_variables[i]))
            f.write("\n\n")

            f.write("# The W matrix represents the agreements activation weights.\n")
            f.write("#\n")
            f.write("# The entire plan can be represented by the W matrix alone,\n")
            f.write("# however, this would imply in loosing the number 0 from the representation.\n")
            f.write("#\n")
            f.write("# For example, you can have an agreement [1,1,1] and the weights [0,0,0], which\n")
            f.write("# in abstract math would imply in an instant agreement fulfillment activation.\n")
            f.write("W: [")
            for i in range(len(self.weights)):
                if i == len(self.weights) - 1:
                    f.write("{}]".format(self.weights[i]))
                else:
                    f.write("{}, ".format(self.weights[i]))
            f.write("\n\n")

            f.write("# The first rendezvous location is where all sub-teams\n")
            f.write("# are supposed to meet first.\n")
            f.write("first_rendezvous: {x: 6.5, y: 5.5, z: 0.0}")

        f.close()

    def get_schedule_stats(self):
        invalid = 0
        robots_per_schedule = []
        for schedule in range(self.schedules):
            count = 0
            for robot in range(self.robots):
                decision, _ = self.get_el(robot, schedule)
                count += decision
            robots_per_schedule.append(count)
            if(count < 2):
                invalid += 100.0
        return invalid, np.array(robots_per_schedule)

    def get_participants_index(self, schedule):
        participants = []
        for index in range(len(schedule)):
            if(schedule[index] == 1):
                participants.append(index)
        return participants

    def get_schedule_decision_variables(self, index : int):
        if(index >= self.schedules):
            raise Exception("Invalid schedule.")
        start_index = index * self.robots
        end_index = start_index + self.robots
        return self.decision_variables[start_index:end_index]

    def get_schedule_weights(self, index : int):
        if(index >= self.schedules):
            raise Exception("Invalid schedule.")
        start_index = index * self.robots
        end_index = start_index + self.robots
        return self.weights[start_index:end_index]        

    def mock_weights_evaluation(self):
        fitness = 0.0
        for x in self.weights:
            fitness += x
        return fitness

    def mock_decision_evaluation(self):
        fitness = 0.0
        for x in self.decision_variables:
            fitness += x
        return fitness

    def get_el(self, x, y):
        index = y * self.robots + x
        return self.decision_variables[index], self.weights[index]