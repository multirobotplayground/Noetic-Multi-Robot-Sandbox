/*
 * Noetic-Multi-Robot-Sandbox for multi-robot research using ROS Noetic
 * Copyright (C) 2020 Alysson Ribeiro da Silva
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef SEARCH_ALGORITHMS_H
#define SEARCH_ALGORITHMS_H

#include <stdio.h>
#include <cstring>
#include <cstdlib>
#include <iostream>
#include <exception>
#include <list>
#include <vector>
#include <limits>
#include "nav_msgs/OccupancyGrid.h"
#include "Common.h"

namespace sa {
    void InitOccFrom(nav_msgs::OccupancyGrid& rInput, nav_msgs::OccupancyGrid& rOutput);
    bool IsInBounds(nav_msgs::OccupancyGrid& rInput, Vec2i& rPos);
    bool CheckAny(nav_msgs::OccupancyGrid& rInput, const Vec2i& rStart, const Vec2i& rEnd, const int& rVal);
    void ComputePath(nav_msgs::OccupancyGrid& rOcc, 
                     const Vec2i rStart, 
                     const Vec2i& rEnd, 
                     std::list<Vec2i>& rOutPath);
    void ComputePathWavefront(
        nav_msgs::OccupancyGrid& rInput, 
        const Vec2i& rStart, 
        const Vec2i& rEnd, 
        std::list<Vec2i>& rOutPath);
    void ComputeFrontiers(nav_msgs::OccupancyGrid& rInput,  
                          nav_msgs::OccupancyGrid& rOutput, 
                          std::vector<Vec2i>& rFrontiers);
    void ComputeClusters(nav_msgs::OccupancyGrid& rFrontiersMap, 
                         std::vector<Vec2i>& rFrontiers, 
                         std::vector<std::vector<Vec2i>>& rOutClusters);
    Vec2i ClosestFrontierCluster(const Vec2i& rPos, std::vector<Vec2i>& rCluster);
    Vec2i MedianFrontierCluster(const Vec2i& rPos, std::vector<Vec2i>& rCluster);
    void ComputeCentroids(const Vec2i& rPos, 
                          std::vector<std::vector<Vec2i>>& rClusters, 
                          std::vector<Vec2i>& rOutCentroids);
    void ComputeAverageCentroids(const Vec2i& rPos, 
                        std::vector<std::vector<Vec2i>>& rClusters, 
                        std::vector<Vec2i>& rOutCentroids);
};
#endif