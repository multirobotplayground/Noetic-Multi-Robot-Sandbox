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

from ScheduleIndividual import ScheduleIndividual
import numpy as np
import os

class Job:
    def __init__(self, start, end, length, id, machine):
        self.start = start
        self.end = end
        self.length = length
        self.id = id
        self.machine = machine

    def print(self):
        print("[s:{} e:{} id:{}]".format(self.start, self.end, self.id, self.machine), end=" ")

class RendezvousJobShopSchedule:
    def __init__(self, schedule : ScheduleIndividual):
        self.schedule = schedule
        self.machines = []
        self.ordered_jobs_list = []
        self.ending_times_arr = []
        self.job_lengths = []
        self.job_groups = {}
        self.max_ending_time = 0.0
        for i in range(self.schedule.schedules):
            self.ending_times_arr.append([])
            self.job_groups[i] = []
        for _ in range(self.schedule.robots):
            start_job = Job(0.0,0.0,0.0,-1,-1)
            self.machines.append([start_job])

    def print_job_shop(self):
        for machine in self.machines:
            for job in machine:
                job.print()
            print()

    def get_last_job(self, machine_index):
        machine = self.machines[machine_index]
        return machine[len(machine)-1]

    def get_max_ending_time(self, participants):
        max_ending_time = float('-inf')
        for machine_index in participants:
            ending_time = self.get_last_job(machine_index).end
            if(ending_time > max_ending_time):
                max_ending_time = ending_time
        return max_ending_time
    
    def num_jobs(self, machine):
        return len(self.machines[machine])
    
    def save_file(self, path, filename="schedule"):
        complete_path = os.path.join(path, filename)
        with open(complete_path, 'w') as f:
            f.write("{}\n{}\n".format(self.schedule.schedules,self.schedule.robots))
            for i in range(len(self.ordered_jobs_list)):
                job = self.ordered_jobs_list[i]
                to_write = ""
                if(i < len(self.ordered_jobs_list)-1):
                    to_write = "{} {} {} {}\n".format(job.start, job.end, job.machine, job.id)
                else:
                    to_write = "{} {} {} {}".format(job.start, job.end, job.machine, job.id)
                f.write(to_write)

    def assemble_job_shop(self):
        for x in range(self.schedule.schedules):
            schedule = self.schedule.get_schedule_decision_variables(x)
            weights = self.schedule.get_schedule_weights(x)
            participants = self.schedule.get_participants_index(schedule)

            #
            # Create all jobs here
            #
            #
            for machine in participants:
                # I need this idjustment for initial allocation
                # Need to remember that robots are stuck until others with the same id finish the rendezvous
                # thus the next start time is the maximum between both
                start_time = 0.0
                if(self.num_jobs(machine) > 1):
                    last_job_id = self.get_last_job(machine).id
                    start_time = np.max(self.ending_times_arr[last_job_id])

                weight = weights[machine]
                end = start_time + weight
                new_job = Job(start_time, end, weight, x, machine)
                self.machines[machine].append(new_job)
                self.ordered_jobs_list.append(new_job)
                self.ending_times_arr[x].append(end)
                self.job_lengths.append(weight)
                self.job_groups[x].append(new_job)

                # keep track of the makespan
                if(end > self.max_ending_time):
                    self.max_ending_time = end

    #
    # Some metrics
    #
    #
    def makespan(self):
        return self.max_ending_time
                
    def total_idle_time(self):
        total_idle_time = 0.0
        max_effort = 0.0
        for schedule in range(self.schedule.schedules):
            current_ending_time_array = self.ending_times_arr[schedule]
            if(len(current_ending_time_array) == 0):
                continue
            max_ending_time = np.max(current_ending_time_array)
            for robot in range(len(current_ending_time_array)):
                total_idle_time += max_ending_time - current_ending_time_array[robot]
                max_effort += max_ending_time
        return total_idle_time, max_effort
    
    def max_end(self, jobs):
        max = float("-inf")
        for job in jobs:
            if job.end > max:
                max = job.end
        return max
    
    def estimate_explored_area(self, robots_capacities_per_second):
        explored_area = 0.0
        for job in self.ordered_jobs_list:
            robot_capacity = robots_capacities_per_second[job.machine]
            explored = job.length * robot_capacity
            explored_area += explored
        return explored_area

    def jobs_processing_time(self):
        return np.array(self.job_lengths)