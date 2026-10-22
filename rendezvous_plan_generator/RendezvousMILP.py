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

import sys
from mip import *
from PlotPlan import PlotChart
from ScheduleIndividual import ScheduleIndividual
from RendezvousJobShopSchedule import RendezvousJobShopSchedule

def RendezvousGeneratorMILP(robots=3, 
                            rendezvous_events=5, 
                            mission_duration_minutes=3, 
                            max_robots_in_schedule=2,
                            min_robots_in_schedule=2,
                            min_job_length_minutes=0,
                            weights=[1.0,1.0]):
    robots = robots
    schedules = rendezvous_events
    mission_minutes = mission_duration_minutes

    model = Model(sense=MINIMIZE, solver_name=CBC)
    total_entries_per_matrix = robots * schedules
    decisions = []
    jobs_s = []
    jobs_e = []
    processing_times_distances = []
    highest_ending_in_group = []
    last_assignment_time = []
    idle_time_vars = []
    work_done_err = model.add_var(var_type=CONTINUOUS, name="work_done_err", lb=0)

    big_M = 1000000
    max_robots_in_schedule = max(min_robots_in_schedule, max_robots_in_schedule)
    
    mission_seconds = mission_minutes * 60
    desired_work_done = mission_seconds * robots

    for schedule in range(schedules):
        s = []
        for robot in range(robots):
            name = "d{}{}".format(schedule, robot)
            s.append(model.add_var(var_type=BINARY, name=name))
        decisions.append(s)

    for schedule in range(schedules):
        s = []
        e = []
        for robot in range(robots):
            name_s = "js{}{}".format(schedule,robot)
            name_e = "je{}{}".format(schedule,robot)
            s.append(model.add_var(var_type=CONTINUOUS, lb=0, name=name_s))
            e.append(model.add_var(var_type=CONTINUOUS, lb=0, name=name_e))
        jobs_s.append(s)
        jobs_e.append(e)

        highest_ending_in_group.append(model.add_var(var_type=CONTINUOUS, lb=0, name="highest_in:{}".format(schedule)))

    for i in range(schedules):
        row = []
        for j in range(robots):
            var = model.add_var(var_type=CONTINUOUS, lb=0, name=f"last_time_{i}_{j}")
            row.append(var)
        last_assignment_time.append(row)

    # Create auxiliary variables for idle time per robot per schedule
    for i in range(schedules):
        row = []
        for j in range(robots):
            var = model.add_var(var_type=CONTINUOUS, lb=0, name=f"idle_{i}_{j}")
            row.append(var)
        idle_time_vars.append(row)

    counts = []
    for j in range(robots):
        var = model.add_var(var_type=INTEGER, name=f"robot_{j}_job_count")
        counts.append(var)

    def deviation_constraints(model, schedules, robots, jobs_s, jobs_e, total_entries_per_matrix, processing_times_distances):
        # Calculate mean job length properly 
        mean = xsum((jobs_e[i][j] - jobs_s[i][j]) for i in range(schedules) for j in range(robots)) / total_entries_per_matrix
        
        for i in range(schedules):
            for j in range(robots):
                var = model.add_var(var_type=CONTINUOUS, lb=0, name="distance{}{}".format(i,j))
                model += var >= (jobs_e[i][j] - jobs_s[i][j]) - mean
                model += var >= -((jobs_e[i][j] - jobs_s[i][j]) - mean)
                processing_times_distances.append(var)

    def big_M_highest_ending_constraints(model, schedules, robots, jobs_e, decisions, highest_ending_in_group, M):
        for i in range(schedules):
            for j in range(robots):
                model += highest_ending_in_group[i] >= jobs_e[i][j] - (1 - decisions[i][j]) * M     

    def min_robots_per_schedule_constraints(model, schedules, min_robots_in_schedule, decision):
        for i in range(schedules):
            model += xsum(decision[i]) >= min_robots_in_schedule

    def max_robots_in_schedule_constraints(model, schedules, max_robots_in_schedule, decision):
        for i in range(schedules):
            model += xsum(decision[i]) <= max_robots_in_schedule

    def job_start_constraints(model, schedules, robots, jobs_s, decisions, highest_ending_in_group, last_assignment_time, M):              
        # Set the last assignment time for each robot at each schedule
        for i in range(schedules):
            for j in range(robots):
                if i == 0:
                    # First schedule: last assignment time is 0
                    model += last_assignment_time[i][j] == 0
                    model += jobs_s[i][j] == 0
                else:
                    #
                    # if else logic
                    # if the robot participated in the last assignment it sets the current as the last
                    # otherwise it propagates the last value forwards... in the last_assignment_time
                    #
                    model += last_assignment_time[i][j] >= last_assignment_time[i-1][j] - M * decisions[i-1][j]
                    model += last_assignment_time[i][j] <= last_assignment_time[i-1][j] + M * decisions[i-1][j]
                    model += last_assignment_time[i][j] >= highest_ending_in_group[i-1] - M * (1 - decisions[i-1][j])
                    model += last_assignment_time[i][j] <= highest_ending_in_group[i-1] + M * (1 - decisions[i-1][j])
                    
                    # Job start constraints: robot can only start after its last assignment time
                    model += jobs_s[i][j] >= last_assignment_time[i][j] - M * (1 - decisions[i][j])
    
    def compute_work_done(schedules, robots, jobs_s, jobs_e):
        return xsum((jobs_e[i][j] - jobs_s[i][j]) for i in range(schedules) for j in range(robots))
    
    def compute_deviation_between_processing_times(schedules, processing_times_deviation, robots):
        return xsum(processing_times_deviation) / (schedules * robots)

    def work_done_err_absolue_value_constraint(model, schedules, robots, jobs_s, jobs_e, desired_work_done):
        model += work_done_err >= compute_work_done(schedules, robots, jobs_s, jobs_e) - desired_work_done
        model += work_done_err >= -(compute_work_done(schedules, robots, jobs_s, jobs_e) - desired_work_done)
    
    def minimum_job_duration_constraints(model, schedules, robots, jobs_s, jobs_e, decisions, min_duration=60):
        for i in range(schedules):
            for j in range(robots):
                model += (jobs_e[i][j] - jobs_s[i][j]) >= min_duration * decisions[i][j]

    def job_duration_constraints(model, schedules, robots, jobs_s, jobs_e, M):
        """Constraints that ensure job durations are non-negative"""
        for i in range(schedules):
            for j in range(robots):
                model += jobs_e[i][j] >= jobs_s[i][j]
                model += jobs_e[i][j] <= mission_seconds
                model += jobs_s[i][j] <= M * decisions[i][j]
                model += jobs_e[i][j] <= M * decisions[i][j]

    def min_number_of_jobs_per_robot_constraints(model, schedules, robots, decisions):
        """Ensure each robot has at least one job assigned"""
        for j in range(robots):
            model += xsum(decisions[i][j] for i in range(schedules)) >= 1

    min_number_of_jobs_per_robot_constraints(model, schedules, robots, decisions)
    min_robots_per_schedule_constraints(model, schedules, min_robots_in_schedule, decisions)
    max_robots_in_schedule_constraints(model, schedules, max_robots_in_schedule, decisions)
    big_M_highest_ending_constraints(model, schedules, robots, jobs_e, decisions, highest_ending_in_group, M=big_M)
    job_start_constraints(model, schedules, robots, jobs_s, decisions, highest_ending_in_group, last_assignment_time, M=big_M)
    job_duration_constraints(model, schedules, robots, jobs_s, jobs_e, M=big_M)
    deviation_constraints(model, schedules, robots, jobs_s, jobs_e, total_entries_per_matrix, processing_times_distances)
    minimum_job_duration_constraints(model, schedules, robots, jobs_s, jobs_e, decisions, min_duration=60*min_job_length_minutes)

    # objetive
    work_done_err_absolue_value_constraint(model, schedules, robots, jobs_s, jobs_e, desired_work_done)

    model.max_gap = 0.10
    model.objective = weights[0] * work_done_err + \
                      weights[1] * compute_deviation_between_processing_times(schedules, processing_times_distances, robots)

    status = model.optimize(max_seconds=30)

    if status == OptimizationStatus.OPTIMAL:
        print('optimal solution cost {} found'.format(model.objective_value))
    elif status == OptimizationStatus.FEASIBLE:
        print('sol.cost {} found, best possible: {}'.format(model.objective_value, model.objective_bound))
    elif status == OptimizationStatus.NO_SOLUTION_FOUND:
        print('no feasible solution found, lower bound is: {}'.format(model.objective_bound))
    if status == OptimizationStatus.OPTIMAL or status == OptimizationStatus.FEASIBLE:
        print('solution:')
        for v in model.vars:
            print('{} : {}'.format(v.name, v.x))

    def extract_rendezvous_schedule(schedules, robots, decisions, jobs_s, jobs_e):
        individual = ScheduleIndividual(robots, schedules)
        individual.set_solution_mips(decisions, jobs_s, jobs_e)
        individual.print_individual()
        return individual
    
    schedule_individual = extract_rendezvous_schedule(schedules, robots, decisions, jobs_s, jobs_e)
    schedule_individual.save_file("./src/multirobotexploration/config/", "rendezvous_plan.yaml")
    rendezvous_jobs = RendezvousJobShopSchedule(schedule_individual)
    rendezvous_jobs.assemble_job_shop()
    rendezvous_jobs.print_job_shop()
    rendezvous_jobs.save_file("./src/multirobotexploration/config/", "mips.jobs")
    PlotChart("./src/multirobotexploration/config/", "mips.jobs")

if __name__ == '__main__':
    """ Example usage of the function
        You can adjust the parameters as needed
        RendezvousGeneratorMILP(robots=3, rendezvous_events=5, mission_duration_minutes=3, max_robots_in_schedule=2, min_job_length_minutes=0, alpha=1.0, beta=1.0)
    """
    if(len(sys.argv) < 8):
        print("Usage: python RendezvousMILP.py "
        "<robots> "
        "<rendezvous_events> "
        "<mission_duration_minutes> "
        "<max_robots_in_schedule> "
        "<min_job_length_minutes> "
        "<alpha> "
        "<beta>")
        sys.exit(1)

    robots = int(sys.argv[1])
    rendezvous_events = int(sys.argv[2])
    mission_duration_minutes = int(sys.argv[3])
    max_robots_in_schedule = int(sys.argv[4])
    min_job_length_minutes = int(sys.argv[5])
    alpha = float(sys.argv[6])
    beta = float(sys.argv[7])

    if robots < 2:
        print("Number of robots must be at least 2.")
        sys.exit(1)

    if rendezvous_events < robots - 1:
        print("Number of rendezvous events must be at least robots - 1.")
        sys.exit(1)

    if mission_duration_minutes < 2:
        print("Mission duration must be at least 2 minutes.")
        sys.exit(1)

    if max_robots_in_schedule < 2 or max_robots_in_schedule > robots - 1:
        print("Max robots in schedule must be at least 2 and at most robots - 1.")
        sys.exit(1)

    if min_job_length_minutes < 0:
        print("Min job length in minutes must be greater than 0.")
        sys.exit(1)

    if alpha < 1.0:
        print("Alpha must be positive.")
        sys.exit(1)
    
    if beta < 1.0:
        print("Beta must be positive.")
        sys.exit(1)

    weights = [alpha, beta]
    RendezvousGeneratorMILP(robots=robots, 
                            rendezvous_events=rendezvous_events, 
                            mission_duration_minutes=mission_duration_minutes, 
                            max_robots_in_schedule=max_robots_in_schedule,
                            min_job_length_minutes=min_job_length_minutes,
                            weights=weights)