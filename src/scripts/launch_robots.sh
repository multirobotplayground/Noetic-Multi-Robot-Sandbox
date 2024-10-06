#!/bin/bash

# Noetic-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
# Copyright (C) 2023 Alysson Ribeiro da Silva
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

# run each stack part in a separate session
tmux new-window -n robot_0_mre_stack -t simulation -d "roslaunch multirobotexploration exploration_stack_bringup.launch robot_id:=0 robot_count:=3"
tmux new-window -n robot_0_gmapping -t simulation -d "roslaunch multirobotexploration gmapping.launch robot_id:=0"

tmux new-window -n robot_1_mre_stack -t simulation -d "roslaunch multirobotexploration exploration_stack_bringup.launch robot_id:=1 robot_count:=3"
tmux new-window -n robot_1_gmapping -t simulation -d "roslaunch multirobotexploration gmapping.launch robot_id:=1"

tmux new-window -n robot_2_mre_stack -t simulation -d "roslaunch multirobotexploration exploration_stack_bringup.launch robot_id:=2 robot_count:=3"
tmux new-window -n robot_2_gmapping -t simulation -d "roslaunch multirobotexploration gmapping.launch robot_id:=2"