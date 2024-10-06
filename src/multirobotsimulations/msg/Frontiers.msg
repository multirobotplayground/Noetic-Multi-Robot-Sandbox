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

geometry_msgs/PoseArray centroids
std_msgs/Float64MultiArray costs
std_msgs/Float64MultiArray values
std_msgs/Float64MultiArray utilities

uint8 highest_cost_index
float32 highest_cost

uint8 lowest_cost_index
float32 lowest_cost

uint8 highest_value_index
float32 highest_value

uint8 lowest_value_index
float32 lowest_value

uint8 highest_utility_index
float32 highest_utility

uint8 lowest_utility_index
float32 lowest_utility