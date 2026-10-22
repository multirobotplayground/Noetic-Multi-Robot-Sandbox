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

# create session with robot_$1 name and in detached mode
# tmux new-session -n global_terminal -s motherbase -d "/bin/bash"

demo=large_map
launch=large_map.launch

if [ ${demo} == "large_map" ]; then
    launch=large_map.launch
    world=sandbox_large_scale.world
elif [ ${demo} == "sandbox" ]; then
    launch=sandbox.launch
    world=sandbox.world
fi

cmd="roslaunch multirobotsimulations ${launch} \
        world_name:=${world} \
        paused:=true \
        for_training:=false"

tmux new-session -n session_manager -s simulation -d ${cmd}

# create new gnome-terminal and attach it to this session
gnome-terminal -t simulation_terminal --geometry=150x20 --hide-menubar -- tmux attach-session -t simulation

