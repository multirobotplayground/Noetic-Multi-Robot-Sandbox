#!/bin/bash

# run each stack part in a separate session
tmux new-window -n robot_0_mre_stack -t simulation -d "roslaunch multirobotexploration exploration_stack_bringup.launch robot_id:=0 robot_count:=3"
tmux new-window -n robot_0_gmapping -t simulation -d "roslaunch multirobotexploration gmapping.launch robot_id:=0"

tmux new-window -n robot_1_mre_stack -t simulation -d "roslaunch multirobotexploration exploration_stack_bringup.launch robot_id:=1 robot_count:=3"
tmux new-window -n robot_1_gmapping -t simulation -d "roslaunch multirobotexploration gmapping.launch robot_id:=1"

tmux new-window -n robot_2_mre_stack -t simulation -d "roslaunch multirobotexploration exploration_stack_bringup.launch robot_id:=2 robot_count:=3"
tmux new-window -n robot_2_gmapping -t simulation -d "roslaunch multirobotexploration gmapping.launch robot_id:=2"