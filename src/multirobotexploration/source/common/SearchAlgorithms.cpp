/*
 * Copyright (c) 2020, Alysson Ribeiro da Silva
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 
 * 3. All advertising materials mentioning features or use of this software must
 *    display the following acknowledgement:
 *    This product includes software developed by Alysson Ribeiro da Silva.
 * 
 * 4. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 * 
 * 5. The source and the binary form, and any modifications made to them
 *    may not be used for the purpose of training or improving machine learning
 *    algorithms, including but not limited to artificial intelligence, natural
 *    language processing, or data mining. This condition applies to any derivatives,
 *    modifications, or updates based on the Software code. Any usage of the source
 *    or the binary form in an AI-training dataset is considered a breach of
 *    this License.
 * 
 * THIS SOFTWARE IS PROVIDED BY COPYRIGHT HOLDER "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL COPYRIGHT HOLDER BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include "SearchAlgorithms.h"
#include "ros/ros.h"
#include <queue>
#include <set>

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
        double dist;
        bool visi;
        MatrixEl() {
            pred = Vec2i::Create(-1,-1);
            dist = std::numeric_limits<double>::max();
            visi = false;
        }
    };

    void ComputePathWavefront(
        nav_msgs::OccupancyGrid& rInput, 
        const Vec2i& rStart, 
        const Vec2i& rEnd, 
        std::list<Vec2i>& rOutPath) {
        // ensure the new path is clear to avoid
        // finding something that does not exists
        // in the frontier discovery
        rOutPath.clear();

        // initialize distances and predecessors
        // using struct with all elements to optimize
        // cache hits
        Matrix<MatrixEl> control(rInput.info.height, rInput.info.width);
        control.clear(MatrixEl());

        std::queue<Vec2i> to_visit;
        to_visit.push(Vec2i::Create(rStart.x, rStart.y));
        control[rStart.y][rStart.x].visi = true;

        Vec2i current;
        Vec2i children;
        bool found = false;
        int index;
        int val;
        MatrixEl* curel;

        while(to_visit.size() > 0) {
            current = to_visit.front();
            to_visit.pop();

            // stop condition
            if(current == rEnd) {
                found = true;
                break;
            }

            // iterate over children
            for(int x = 0; x < 3; ++x) {
                for(int y = 0; y < 3; ++y) {
                    if(x == y) continue;
                    children = Vec2i::Create(current.x - 1 + x, current.y - 1 + y);
                    // only add children that were not visited
                    if(IsInBounds(rInput, children) && control[children.y][children.x].visi == false) {
                        index = children.y*rInput.info.width+children.x;
                        val = rInput.data[index];
                        // check if the OCC is clean regarding data
                        if(val >= 0 && val < 90) {
                            curel = &(control[children.y][children.x]);
                            curel->visi = true;
                            curel->pred = Vec2i::Create(current.x, current.y);
                            to_visit.push(Vec2i::Create(children.x, children.y));
                       }
                    }
                }
            }
        }

        // compute output path from search
        if(found == true) {
            current = rEnd;
            while(current != rStart) {
                rOutPath.push_front(current);
                current = control[current.y][current.x].pred;
            }
        }
    }

    void ComputePath(
        nav_msgs::OccupancyGrid& rInput, 
        const Vec2i& rStart, 
        const Vec2i& rEnd, 
        std::list<Vec2i>& rOutPath) {

        // ensure the new path is clear to avoid
        // finding something that does not exists
        // in the frontier discovery
        rOutPath.clear();

        // initialize distances and predecessors
        // using struct with all elements to optimize
        // cache hits
        Matrix<MatrixEl> control(rInput.info.height, rInput.info.width);
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
        q.push(el(Vec2i::Create(rStart.x, rStart.y), 0.0));
        control[rStart.y][rStart.x].dist = 0.0;

        // do search
        while(q.size() > 0) {
            // TODO:swap the extract min function to 
            // be more optimized
            current = q.top().pos;
            q.pop();

            if(control[current.y][current.x].visi == true) continue;
            control[current.y][current.x].visi = true;

            // stop condition
            if(current == rEnd) {
                found = true;
                break;
            }

            // iterate over children
            for(int x = 0; x < 3; ++x) {
                for(int y = 0; y < 3; ++y) {
                    if(x == 1 && y == 1) continue;

                    children = Vec2i::Create(current.x - 1 + x, current.y - 1 + y);
                    if(IsInBounds(rInput, children)
                       && control[children.y][children.x].visi == false 
                       && rInput.data[children.y*rInput.info.width+children.x] >= 0
                       && rInput.data[children.y*rInput.info.width+children.x] < 50) {
                        // compute distance and heuristic to parent
                        // and end points
                        dist = control[current.y][current.x].dist + Distance(current, children);
                        heuristic = Distance(rEnd, children);
                        distance_metric = dist + heuristic;
                        
                        // relax edge
                        if(control[children.y][children.x].dist > distance_metric) {
                            control[children.y][children.x].dist = distance_metric;
                            control[children.y][children.x].pred = Vec2i::Create(current.x, current.y);
                        } 

                        // if not on the heap, add to the heap
                        el element(children, control[children.y][children.x].dist);
                        q.push(element);
                    }
                }
            }
        }

        // compute output path from search
        if(found) {
            current = rEnd;
            while(current != rStart) {
                rOutPath.push_front(current);
                current = control[current.y][current.x].pred;
            }
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
                if(cluster.size() > 0) {
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