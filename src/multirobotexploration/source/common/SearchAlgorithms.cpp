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

#include "SearchAlgorithms.h"
#include "ros/ros.h"
#include <queue>
#include <set>
#include <cmath>

std::mt19937* randomglobal() {
    static std::random_device rd;
    static std::mt19937 gen(rd());
    return &gen;
}

namespace sa {

    struct el{
        Vec2i pos;
        double dist;
        el(){
            pos = Vec2i::Create(0,0);
            dist = 0.0;
        }
        el(const Vec2i p, const double d) {
            pos = p;
            dist = d;
        }
        bool operator<(const el& a) const {
            return dist > a.dist;
        }
    };

    void InitOccFrom(nav_msgs::OccupancyGrid& rInput, nav_msgs::OccupancyGrid& rOutput) {
        rOutput.data.assign(rInput.data.size(), -1);
        rOutput.info = rInput.info;
        rOutput.header = rInput.header;
    }

    bool IsInBounds(nav_msgs::OccupancyGrid& rInput, Vec2i& rPos) {
        return rPos.x >= 0 &&
            rPos.y >= 0 &&
            rPos.x < rInput.info.width &&
            rPos.y < rInput.info.height;
    }

    bool CheckAny(nav_msgs::OccupancyGrid& rInput, const Vec2i& rStart, const Vec2i& rEnd, const int& rVal) {
        Vec2i start = rStart;
        Vec2i end = rEnd;
        int width = rInput.info.width;
        int height = rInput.info.height;
        if(start.x < 0) start.x = 0;
        if(start.x >= width) start.x = width - 1;
        if(start.y < 0) start.y = 0;
        if(start.y >= height) start.y = height - 1;
        if(end.x < 0) end.x = 0;
        if(end.x > width) end.x = width;
        if(end.y < 0) end.y = 0;
        if(end.y > height) end.y = height;

        // this for should be end inclusive
        // thus is the end.[xy] + 1
        for(int y = start.y; y < end.y + 1; ++y) {
            for(int x = start.x; x < end.x + 1; ++x) {
                if(rInput.data[y*width+x] == rVal) return true;
            }
        }
        return false;
    }

    /*
    * ComputePath is an implementation of the A* algorithm on top of a cell decomposed map.
    * where black pixels are free space, white pixels are obstacles, and
    * blue pixels are frontiers.
    */
    struct MatrixEl{
        Vec2i pred;
        double g_score;  // best distance from start
        bool processed;  // in closed set (already processed)
        MatrixEl() {
            pred = Vec2i::Create(-1,-1);
            g_score = std::numeric_limits<double>::max();
            processed = false;
        }
    };

    void ComputePath(
        nav_msgs::OccupancyGrid& rInput, 
        const Vec2i rStart, 
        const Vec2i& rEnd, 
        std::list<Vec2i>& rOutPath) {

        // ensure the new path is clear to avoid
        // finding something that does not exists
        // in the frontier discovery
        rOutPath.clear();

        // validate input bounds
        if(!IsInBounds(rInput, const_cast<Vec2i&>(rStart)) || 
           !IsInBounds(rInput, const_cast<Vec2i&>(rEnd))) {
            ROS_DEBUG("[SearchAlgorithms] Start or end position out of bounds");
            return;
        }

        // check if start and end are passable (use consistent threshold)
        int start_idx = rStart.y*rInput.info.width+rStart.x;
        int end_idx = rEnd.y*rInput.info.width+rEnd.x;
        
        if(rInput.data[start_idx] > 50 || rInput.data[start_idx] < 0 ||
           rInput.data[end_idx] > 50 || rInput.data[end_idx] < 0) {
            ROS_DEBUG("[SearchAlgorithms] Start (%d,%d) or end (%d,%d) position not passable. Start val: %d, End val: %d", 
                     rStart.x, rStart.y, rEnd.x, rEnd.y, rInput.data[start_idx], rInput.data[end_idx]);
            return;
        }

        // If start equals end, return immediately
        if(rStart == rEnd) {
            return;
        }

        // initialize distances and predecessors
        // using struct with all elements to optimize
        // cache hits
        Matrix<MatrixEl> control(rInput.info.width, rInput.info.height);
        control.clear(MatrixEl());

        // control variables
        bool found = false;
        double dist = 0.0;
        double heuristic = 0.0;
        double distance_metric = 0.0;
        Vec2i current;
        Vec2i children;

        // initialize search
        std::priority_queue<el> q;
        double initial_f = sqrt(Distance(rStart, rEnd));
        q.push(el(Vec2i::Create(rStart.x, rStart.y), initial_f));
        control[rStart.y][rStart.x].g_score = 0.0;
        control[rStart.y][rStart.x].pred = Vec2i::Create(-1, -1); // No predecessor

        // Limit iterations to prevent infinite loops
        int max_iterations = rInput.info.width * rInput.info.height;
        int iterations = 0;

        // do search
        while(q.size() > 0 && iterations < max_iterations) {
            iterations++;
            
            current = q.top().pos;
            q.pop();
            
            // Skip if already processed
            if(control[current.y][current.x].processed) {
                continue;
            }
            
            // Mark as processed (closed set)
            control[current.y][current.x].processed = true;

            // stop condition
            if(current == rEnd) {
                found = true;
                break;
            }

            // iterate over children (8-directional movement)
            for(int x = 0; x < 3; ++x) {
                for(int y = 0; y < 3; ++y) {
                    if(x == 1 && y == 1) continue;

                    children = Vec2i::Create(current.x - 1 + x, current.y - 1 + y);
                    if(IsInBounds(rInput, children)
                       && !control[children.y][children.x].processed) {
                        
                        int child_idx = children.y*rInput.info.width+children.x;
                        int child_val = rInput.data[child_idx];
                        
                        // Use consistent obstacle threshold (50) and check for unknown cells
                        if(child_val <= 50) {
                            // compute tentative g_score
                            double edge_cost = sqrt(Distance(current, children));
                            double tentative_g = control[current.y][current.x].g_score + edge_cost;
                            
                            // If this path to neighbor is better than any previous one
                            if(tentative_g < control[children.y][children.x].g_score) {
                                // Record the better path
                                control[children.y][children.x].g_score = tentative_g;
                                control[children.y][children.x].pred = Vec2i::Create(current.x, current.y);
                                
                                // Add to priority queue with f-score
                                heuristic = sqrt(Distance(rEnd, children));
                                double f_score = tentative_g + heuristic;
                                q.push(el(children, f_score));
                            } 
                        }
                    }
                }
            }
        }

        if(iterations >= max_iterations) {
            ROS_WARN("[SearchAlgorithms] A* search exceeded maximum iterations (%d)", max_iterations);
        }

        // compute output path from search
        if(found) {
            current = rEnd;
            int path_length = 0;
            int max_path_length = rInput.info.width + rInput.info.height; // Reasonable max path length
            
            while(current != rStart && path_length < max_path_length) {
                rOutPath.push_front(current);
                Vec2i next = control[current.y][current.x].pred;
                
                // Validate predecessor to prevent infinite loops
                if(next.x == -1 && next.y == -1) {
                    ROS_WARN("[SearchAlgorithms] Invalid predecessor found during path reconstruction");
                    rOutPath.clear();
                    return;
                }
                
                current = next;
                path_length++;
            }
            
            if(path_length >= max_path_length) {
                ROS_WARN("[SearchAlgorithms] Path reconstruction exceeded maximum length");
                rOutPath.clear();
            } else {
                ROS_DEBUG("[SearchAlgorithms] Found path with %zu waypoints in %d iterations", rOutPath.size(), iterations);
            }
        } else {
            ROS_DEBUG("[SearchAlgorithms] No path found from (%d,%d) to (%d,%d) after %d iterations", 
                     rStart.x, rStart.y, rEnd.x, rEnd.y, iterations);
        }
    }

    void ComputeFrontiers(nav_msgs::OccupancyGrid& rInput, 
                          nav_msgs::OccupancyGrid& rOutput, 
                          std::vector<Vec2i>& rFrontiers) {
        rFrontiers.clear();                            
        InitOccFrom(rInput, rOutput);
        Vec2i start;
        Vec2i end;
        int index;
        for(int y = 0; y < rInput.info.height; ++y) {
            for(int x = 0; x < rInput.info.width; ++x) {
                start.x = x - 1; start.y = y - 1;
                end.x   = x + 1; end.y   = y + 1;

                // only consider frontiers that are reachable
                index = y * rInput.info.width + x;
                if(rInput.data[index] >= 0 
                   && rInput.data[index] < 50 
                   && CheckAny(rInput, start, end, -1)) {
                    rOutput.data[index] = 100;
                    rFrontiers.push_back(Vec2i::Create(x,y));
                }
            }
        }
    }

    void ComputeClusters(nav_msgs::OccupancyGrid& rFrontiersMap, 
                        std::vector<Vec2i>& rFrontiers, 
                        std::vector<std::vector<Vec2i>>& rOutClusters) {
        Matrix<int> visited(rFrontiersMap.info.width, rFrontiersMap.info.height);
        visited.clear(0);
        rOutClusters.clear();

        std::list<Vec2i> q;
        std::vector<Vec2i> cluster;
        Vec2i current;
        Vec2i children;  
        for(size_t f = 0; f < rFrontiers.size(); ++f) {
            current = rFrontiers[f];
            if(visited[current.y][current.x] == 0) {
                q.clear();
                cluster.clear();
                q.push_back(current);
                while(q.size() > 0) {
                    current = q.back();
                    q.pop_back();
                    cluster.push_back(current);
                    visited[current.y][current.x] = 2;
                    for(int y = 0; y < 3; ++y) {
                        for(int x = 0; x < 3; ++x){
                            children = Vec2i::Create(current.x - 1 + x, current.y - 1 + y);
                            if(IsInBounds(rFrontiersMap, children)
                               && visited[children.y][children.x] == 0
                               && rFrontiersMap.data[children.y*rFrontiersMap.info.width+children.x] >= 90) {
                                visited[children.y][children.x] = 1;
                                q.push_back(children);
                            }
                        }
                    }
                }

                // after the flooding search
                // append clusters to clusters list
                if(cluster.size() > 10) {
                    rOutClusters.push_back(cluster);
                }
            }
        }
    }

    Vec2i ClosestFrontierCluster(const Vec2i& rPos, std::vector<Vec2i>& rCluster) {
        int closest = 0;
        double dist = std::numeric_limits<double>::max();
        double temp_dist = 0.0;
        for(size_t i = 0; i < rCluster.size(); ++i) {
            temp_dist = Distance(rCluster[i], rPos);
            if(temp_dist < dist) {
                dist = temp_dist;
                closest = i;
            }
        }
        return Vec2i::Create(rCluster[closest].x, rCluster[closest].y);
    }

    Vec2i MedianFrontierCluster(const Vec2i& rPos, std::vector<Vec2i>& rCluster) {
        std::vector<std::pair<double, Vec2i>> to_sort;
        std::pair<double, Vec2i> min;
        std::pair<double, Vec2i> max;
        min.first = std::numeric_limits<double>::max();
        max.first = std::numeric_limits<double>::min();
        double distance = -1.0;
        double average = 0.0;
        for(size_t i = 0; i < rCluster.size(); ++i) {
            distance = Distance(rCluster[i], rPos);
            to_sort.push_back(std::pair<double, Vec2i>(distance, rCluster[i]));
            if(distance > max.first) {
                max.first = distance;
                max.second = rCluster[i];
            }
            if(distance < min.first) {
                min.first = distance;
                min.second = rCluster[i];
            }
        }
        average = (min.first + max.first)/2.0;
        // seek for near to average
        double dist = std::numeric_limits<double>::max();
        Vec2i near;
        for(size_t i = 0; i < to_sort.size(); ++i) {
            distance = abs(average - to_sort[i].first);
            if(distance < dist) {
                dist = distance;
                near = to_sort[i].second;
            }
        }
        return near;
    }

    void ComputeCentroids(const Vec2i& rPos, 
                        std::vector<std::vector<Vec2i>>& rClusters, 
                        std::vector<Vec2i>& rOutCentroids) {
        rOutCentroids.clear();
        for(size_t i = 0; i < rClusters.size(); ++i) {
            rOutCentroids.push_back(ClosestFrontierCluster(rPos, rClusters[i]));
        }
    }

    void ComputeAverageCentroids(const Vec2i& rPos, 
                        std::vector<std::vector<Vec2i>>& rClusters, 
                        std::vector<Vec2i>& rOutCentroids) {
        rOutCentroids.clear();
        for(size_t i = 0; i < rClusters.size(); ++i) {
            rOutCentroids.push_back(MedianFrontierCluster(rPos, rClusters[i]));
        }
    }
};