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

import matplotlib.pyplot as plt
import sys
import matplotlib.cm as cmaps
import os

def Union(lst1, lst2):
    final_list = list(set(lst1) | set(lst2))
    return final_list

def PlotChart(path, file_name="schedule"):
    fig, axes = plt.subplots(1,1)
    complete_path = os.path.join(path, file_name)
    f = open(complete_path)

    lines = f.read()
    lines = lines.split("\n")
    robots_count = int(lines[1])
    jobs = lines[2:]
    
    robots = [x for x in range(robots_count)]
    starts = []
    ends = []
    alloc = []
    plans = []
    widths = []
    labels_y = []
    robot_names = []

    i = 0
    for job in jobs:
        data = job.split()
        start = float(data[0]) / 60.0  # Convert seconds to minutes
        end = float(data[1]) / 60.0    # Convert seconds to minutes
        width = end - start
        robot = int(data[2])
        plan = int(data[3])

        starts.append(start)
        ends.append(end)
        alloc.append(robot)
        plans.append(plan)
        widths.append(width)

        i+=1

    for i in robots:
        robot_names.append("Robot {}".format(i))

    alloc_arr = [[] for i in range(robots_count)]
    machine_bars = [[] for i in range(robots_count)]
    machine_y_ranges = [() for i in range(robots_count)]
    colors = [[] for i in range(robots_count)]
    # colors_map = list(mcolors.CSS4_COLORS)
    bar_colors = [[] for i in range(robots_count)]
    
    plan_color_map = []
    plan_color_map = cmaps.get_cmap('tab20c')([x for x in range(20)])
    #plan_color_map = Union(plan_color_map, cmaps.get_cmap('Dark2')([x for x in range(9)]))
    #plan_color_map = Union(plan_color_map, cmaps.get_cmap('Set1')([x for x in range(9)]))
    #plan_color_map = Union(plan_color_map, cmaps.get_cmap('Set2')([x for x in range(9)]))
    #plan_color_map = Union(plan_color_map, cmaps.get_cmap('Set3')([x for x in range(9)]))
    #plan_color_map = Union(plan_color_map, cmaps.get_cmap('tab20b')([x for x in range(9)]))
    #lan_color_map = Union(plan_color_map, cmaps.get_cmap('tab20c')([x for x in range(9)]))
    #plan_color_map = Union(plan_color_map, cmaps.get_cmap('tab20c')([x for x in range(9)]))
    #print(plan_color_map)

    i = 0
    while i < len(jobs):
        machine = alloc[i]
        start = starts[i]
        end = ends[i]
        plan = plans[i]
        robot = []
        length = end - start
        y_start = machine+1.05
        y_height = 0.95
        y_range = (y_start, y_height)

        alloc_arr[machine].append(machine)
        machine_bars[machine].append((start, length))
        machine_y_ranges[machine] = y_range
        colors[machine].append(plan)
        bar_colors[machine].append(plan_color_map[plan])
        i += 1

    for machine in range(len(machine_bars)):
        y_start = machine+1.05
        y_height = 0.95
        labels_y.append(y_start + y_height/2.0)

        ax = axes

    ax.set_xlabel('Time (m)')

    ax.set_yticks(labels_y)
    ax.set_yticklabels(robot_names)
    
    i = 0
    while i < len(machine_bars):
        m_bar = machine_bars[i]
        y_range = machine_y_ranges[i]
        bar_color = bar_colors[i]
        if len(m_bar) == 0:
            m_bar = [(0.0,0.0)]
            y_range = (1.05, 0.95)

        ax.broken_barh(m_bar, y_range, facecolors=bar_color, edgecolor='black')
        i += 1

    f.close()
    plt.savefig(complete_path+".png")
    plt.show()

if __name__ == "__main__":
    PlotChart(str(sys.argv[1]), str(sys.argv[2]))
    
