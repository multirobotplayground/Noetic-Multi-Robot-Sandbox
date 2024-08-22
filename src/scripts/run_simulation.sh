#!/bin/bash

# create session with robot_$1 name and in detached mode
# tmux new-session -n global_terminal -s motherbase -d "/bin/bash"
tmux new-session -n session_manager -s simulation -d "roslaunch multirobotsimulations gazebo_multi_robot_bringup.launch world_name:=forest.world paused:=true"

# create new gnome-terminal and attach it to this session
# gnome-terminal -t motherbase_terminal --geometry=100x100 --hide-menubar -- tmux attach-session -t motherbase
gnome-terminal -t simulation_terminal --geometry=150x20 --hide-menubar -- tmux attach-session -t simulation