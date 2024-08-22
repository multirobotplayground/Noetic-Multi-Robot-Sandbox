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

#ifndef COMMON_H
#define COMMON_H

#include <cstring>
#include <stdio.h>
#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "nav_msgs/OccupancyGrid.h"
#include <random>

std::mt19937* randomglobal();

/*
 * For grid opperations
 */
struct Vec2i {
	union {
		int array[2];
		struct {
			union {
				int x;
				int width;
			};
			union {
				int y;
				int height;
			};
		};
	};

	static Vec2i Create(int x, int y) {
		Vec2i result;
		result.x = x;
		result.y = y;
		return result;
	}

	static Vec2i Create(int v) {
		Vec2i result;
		result.x = v;
		result.y = v;
		return result;
	}

	bool contains(int v) const {
		return x == v || y == v;
	}

	bool contains(const Vec2i& v) const {
		return contains(v.x) || contains(v.y);
	}

	inline void print() {
		printf("[%d,%d]\n", x, y);
	}

	inline bool operator==(const Vec2i& v) const { return memcmp(this, &v, sizeof(Vec2i)) == 0; }
	inline bool operator!=(const Vec2i& v) const { return memcmp(this, &v, sizeof(Vec2i)) != 0; }
	inline Vec2i operator-()const { return Vec2i::Create(-x, -y); }
	inline Vec2i& operator+=(const Vec2i& v) { x += v.x; y += v.y; return (*this); }
	inline Vec2i& operator-=(const Vec2i& v) { x -= v.x; y -= v.y; return (*this); }
	inline Vec2i& operator*=(const Vec2i& v) { x *= v.x; y *= v.y; return (*this); }
	inline Vec2i& operator/=(const Vec2i& v) { x /= v.x; y /= v.y; return (*this); }
	inline Vec2i& operator+=(const int& v) { x += v; y += v; return (*this); }
	inline Vec2i& operator-=(const int& v) { x -= v; y -= v; return (*this); }
	inline Vec2i& operator*=(const int& v) { x *= v; y *= v; return (*this); }
	inline Vec2i& operator/=(const int& v) { x /= v; y /= v; return (*this); }
	inline double norm(const Vec2i& other) {
		int dx = this->x - other.x;
		int dy = this->y - other.y;
		return sqrt((dx * dx) + (dy * dy));
	}

};

inline double Distance(const Vec2i& rA, const Vec2i& rB) {
	int dx = rA.x - rB.x;
	int dy = rA.y - rB.y;
	return (dx * dx) + (dy * dy);
}

#define INLINE_OPERATION_IMPLEMENTATION_INT(TTYPE)\
static inline TTYPE operator/( const TTYPE& vecA, const TTYPE& vecB ){ return (TTYPE(vecA)/=vecB); } \
static inline TTYPE operator/( const TTYPE& vec , const int value ){ return (TTYPE(vec)/=value); } \
static inline TTYPE operator/( const int value, const TTYPE& vec  ){ return (TTYPE::Create(value)/=vec); } \
static inline TTYPE operator*( const TTYPE& vecA, const TTYPE& vecB ){ return (TTYPE(vecA)*=vecB); } \
static inline TTYPE operator*( const TTYPE& vec , const int value ){ return (TTYPE(vec)*=value); } \
static inline TTYPE operator*( const int value, const TTYPE& vec  ){ return (TTYPE::Create(value)*=vec); } \
static inline TTYPE operator+( const TTYPE& vecA, const TTYPE& vecB ){ return (TTYPE(vecA)+=vecB); } \
static inline TTYPE operator+( const TTYPE& vec , const int value ){ return (TTYPE(vec)+=value); } \
static inline TTYPE operator+( const int value, const TTYPE& vec  ){ return (TTYPE::Create(value)+=vec); } \
static inline TTYPE operator-( const TTYPE& vecA, const TTYPE& vecB ){ return (TTYPE(vecA)-=vecB); } \
static inline TTYPE operator-( const TTYPE& vec , const int value ){ return (TTYPE(vec)-=value); } \
static inline TTYPE operator-( const int value, const TTYPE& vec  ){ return (TTYPE::Create(value)-=vec); }

INLINE_OPERATION_IMPLEMENTATION_INT(Vec2i)

#if !defined(NDEBUG)
      #define MATRIX_THROW_OUT_OF_BOUND_EXCEPTION
#endif

#if defined(MATRIX_THROW_OUT_OF_BOUND_EXCEPTION)

template <typename T>
class Matrix;

template <typename _T>
class Matrix_Proxy {
    int width;
    _T* ptr;
    Matrix_Proxy(int width, _T* ptr) {
        this->width = width;
        this->ptr = ptr;
    }
    public:
    _T& operator[](int x) {
        if (x < 0 || x >= width)
            throw std::overflow_error("column index is out of bounds.");
        return this->ptr[x];
    }
    const _T& operator[](int x)const {
        if (x < 0 || x >= width)
            throw std::overflow_error("column index is out of bounds.");
        return this->ptr[x];
    }
    friend class Matrix<_T>;
};
#endif

/*
 * For grid opperations
 */
template <typename T> class Matrix {
	public:
		T* array;
		Vec2i size;

	Matrix(int width, int height) : size(Vec2i::Create(0,0)) {
		array = NULL;
		setSize(Vec2i::Create(width, height));
	}

	Matrix(const Vec2i& size) : size(Vec2i::Create(0,0)) {
		array = NULL;
		setSize(size);
	}

	Matrix(const Matrix& m) : size(Vec2i::Create(0,0)) {
		setSize(m.size);
		memcpy(array, m.array, sizeof(T)*size.width*size.height);
	}

	void operator=(const Matrix& m) {
		setSize(m.size);
		memcpy(array, m.array, sizeof(T)*size.width*size.height);
	}

	inline void CopyRegion(std::vector<Vec2i>& rValidAreas,
						   Matrix<T>& pOther, 
						   const Vec2i& start, 
						   const Vec2i& end) {
		if(size != pOther.size) {
			throw std::out_of_range("matrices need to be of the same size.");
		}
		if(start.x < 0 || start.x > size.width || end.x < 0 || end.x > size.width ||
		   start.y < 0 || start.y > size.height || end.y < 0 || end.y > size.height) {
			throw std::out_of_range("box is out of range.");
		}
		int index = 0;
		int val = 0;

		// should include both ends, thus using <=
		for(int y = start.y; y < end.y; ++y) {
			// this copy is being done this way to extract all 
			// free positions that will be used for map fusion
			// thus reducing the algorithmic complexity of the program
			for(int x = start.x; x < end.x; ++x) {
				index = y*size.width+x;
				val = pOther.array[index];

				// if val is a valid free path,
				// them it should be added into the free
				// paths list if it was not discovered 
				// into the robot's map
				if(val != 50 && array[index] == 50) {
					rValidAreas.push_back(Vec2i::Create(x,y));
				}

				// the value will be updated only after checking
				// its discovery on the robot's map
				array[index] = val;
			}
			
			// direct copy is efficient but does not allow fast 
			// map fusion since there is no way to get the free positions
			// directly from the memory block
			//index = y*size.width+x;
			//memcpy(&array[index], &pOther.array[index], sizeof(T)*(end.x-start.x));
		}
	}

	inline bool CheckAny(const Vec2i& rStart, const Vec2i& rEnd, const int& rVal) {
		Vec2i start = rStart;
		Vec2i end = rEnd;
		if(start.x < 0) start.x = 0;
		if(start.x >= size.width) start.x = size.width - 1;
		if(start.y < 0) start.y = 0;
		if(start.y >= size.height) start.y = size.height - 1;
		if(end.x < 0) end.x = 0;
		if(end.x >= size.width) end.x = size.width - 1;
		if(end.y < 0) end.y = 0;
		if(end.y >= size.height) end.y = size.height - 1;

		// this for should be end inclusive
		// thus is the end.[xy] + 1
		for(int y = start.y; y < end.y + 1; ++y) {
			for(int x = start.x; x < end.x + 1; ++x) {
				if(array[y*size.width+x] == rVal) return true;
			}
		}
		return false;
	}

	#if defined(MATRIX_THROW_OUT_OF_BOUND_EXCEPTION)
		Matrix_Proxy<T> operator[](int y) {
			if (y < 0 || y >= size.height)
				throw std::overflow_error("row index is out of bounds.");
			return Matrix_Proxy<T>(size.width , &this->array[y * size.width]);
		}
		const Matrix_Proxy<T> operator[](int y)const {
			if (y < 0 || y >= size.height)
				throw std::overflow_error("row index is out of bounds.");
			return Matrix_Proxy<T>(size.width, &this->array[y * size.width]);
		}
	#else
		T* operator[](int y) {
			return &this->array[y * size.width];
		}
		const T* operator[](int y)const {
			return &this->array[y * size.width];
		}
	#endif

	~Matrix() {
		delete array;
		array = nullptr;
		this->size = Vec2i::Create(0);
	}

	void setSize(const Vec2i& size) {
		if (this->size == size)
			return;
		if(size.width <= 0 || size.height <= 0) {
			throw std::bad_alloc();
		} 
		delete array;
		array = nullptr;
		this->size = size;
		array = new T[size.width * size.height];
	}

	bool isInBounds(const Vec2i& rCoord) {
		if(rCoord.x < 0) return false;
		if(rCoord.y < 0) return false;
		if(rCoord.x >= size.width) return false;
		if(rCoord.y >= size.height) return false;
		return true;
	} 

	void clear(const T&v) {
		int s = size.height * size.width;
		for(int i=0;i<s;++i) {
			array[i] = v;
		}
	}
};

inline void PrintIntMap(Matrix<int>& rInput) {
    // print frontier maps for evaluation
    for(int y = 0; y < rInput.size.height; ++y) {
        for(int x = 0; x < rInput.size.width; ++x) {
            if(rInput[y][x] == 100)
                printf("%c", 251);
			else if(rInput[y][x] == 50)
				printf("?");
			else
				printf(".");
        }
        printf("\n");
    }
}

inline void PrintCharMap(Matrix<char>& rInput) {
    // print frontier maps for evaluation
    for(int y = 0; y < rInput.size.height; ++y) {
        for(int x = 0; x < rInput.size.width; ++x) {
            printf("%c", rInput[y][x]);
        }
        printf("\n");
    }
}

inline void PrintSelfMap(Matrix<int>& rInput, const Vec2i& rPos) {
	Matrix<char> cmap(rInput.size);

    // print frontier maps for evaluation
    for(int y = 0; y < rInput.size.height; ++y) {
        for(int x = 0; x < rInput.size.width; ++x) {
            if(rInput[y][x] == 100)
                cmap[y][x] = 251;
			else if(rInput[y][x] == 50)
				cmap[y][x] = '?';
			else
				cmap[y][x] = '.';
        }
    }

	cmap[rPos.y][rPos.x] = 'R';

	PrintCharMap(cmap);
}

inline void PrintIntPath(Matrix<int>& rInput, 
						 std::list<Vec2i>& rPath, 
						 const Vec2i& rStart, 
						 const Vec2i& rEnd) {
	Matrix<char> path_map(rInput.size);

    // print frontier maps for evaluation
    for(int y = 0; y < rInput.size.height; ++y) {
        for(int x = 0; x < rInput.size.width; ++x) {
            if(rInput[y][x] == 100)
                path_map[y][x] = 251;
			else if(rInput[y][x] == 50)
				path_map[y][x] = '?';
			else
				path_map[y][x] = '.';
        }
    }

	// mark path in the path_map
	std::list<Vec2i>::iterator it = rPath.begin();
	for(;it != rPath.end(); ++it) {
		Vec2i p = (*it);
		path_map[p.y][p.x] = '*';
	}

	path_map[rStart.y][rStart.x] = 'S';
	path_map[rEnd.y][rEnd.x] = 'E';

	// print full map
	PrintCharMap(path_map);
}

/*
 * Occupancy grid helpers
 */
inline void PoseToVector3(const geometry_msgs::Pose& rInput, tf::Vector3& rOut) {
	rOut.setX(rInput.position.x);
	rOut.setY(rInput.position.y);
	rOut.setZ(rInput.position.z);
	rOut.setW(1.0);
}

inline void Vector3ToPose(const tf::Vector3& rInput, geometry_msgs::Pose& rOutPose) {
	rOutPose.position.x = rInput.getX();
	rOutPose.position.y = rInput.getY();
	rOutPose.position.z = rInput.getZ();
}

inline void WorldToMap(nav_msgs::OccupancyGrid& rOcc, geometry_msgs::Pose& rWorld, Vec2i& rOutput) {
    double res = rOcc.info.resolution;
    rOutput.x = (int)((rWorld.position.x - rOcc.info.origin.position.x)/res);
    rOutput.y = (int)((rWorld.position.y - rOcc.info.origin.position.y)/res);
}

inline void WorldToMap(nav_msgs::OccupancyGrid& rOcc, geometry_msgs::Pose& rWorld, tf::Vector3& rOutput) {
    double res = rOcc.info.resolution;
    rOutput.setX((int)((rWorld.position.x - rOcc.info.origin.position.x)/res));
    rOutput.setY((int)((rWorld.position.y - rOcc.info.origin.position.y)/res));
}

inline void WorldToMap(nav_msgs::OccupancyGrid& rOcc, tf::Vector3& rWorld, Vec2i& rOutput) {
    double res = rOcc.info.resolution;
    rOutput.x = (int)((rWorld.getX() - rOcc.info.origin.position.x)/res);
    rOutput.y = (int)((rWorld.getY() - rOcc.info.origin.position.y)/res);
}

inline void WorldToMap(nav_msgs::OccupancyGrid& rOcc, tf::Vector3& rWorld, tf::Vector3&rOutput) {
    double res = rOcc.info.resolution;
    rOutput.setX((int)((rWorld.getX() - rOcc.info.origin.position.x)/res));
    rOutput.setY((int)((rWorld.getY() - rOcc.info.origin.position.y)/res));
	rOutput.setZ(0.0);
}

inline void MapToWorld(nav_msgs::OccupancyGrid& rOcc, Vec2i& rMap, geometry_msgs::Pose& rWorld) {
    double res = rOcc.info.resolution;
	double d_x = (double)rMap.x;
	double d_y = (double)rMap.y;
    rWorld.position.x = rOcc.info.origin.position.x + (d_x * res) + (res / 2.0);
    rWorld.position.y = rOcc.info.origin.position.y + (d_y * res) + (res / 2.0);
    rWorld.position.z = 0.0;
}

inline void MapToWorld(nav_msgs::OccupancyGrid& rOcc, Vec2i& rMap, Vec2i& rWorld) {
    double res = rOcc.info.resolution;
	double d_x = (double)rMap.x;
	double d_y = (double)rMap.y;
    rWorld.x = rOcc.info.origin.position.x + (d_x * res) + (res / 2.0);
    rWorld.y = rOcc.info.origin.position.y + (d_y * res) + (res / 2.0);
}

inline void MapToWorld(nav_msgs::OccupancyGrid& rOcc, Vec2i& rMap, tf::Vector3& rWorld) {
    double res = rOcc.info.resolution;
	double d_x = (double)rMap.x;
	double d_y = (double)rMap.y;
    rWorld.setX(rOcc.info.origin.position.x + (d_x * res) + (res / 2.0));
    rWorld.setY(rOcc.info.origin.position.y + (d_y * res) + (res / 2.0));
}

inline double PeriodToFreqAndFreqToPeriod(const double& val) {
	return 1.0 / val;
}

inline void ApplyMask(const int& rX, 
                const int& rY, 
                const int& rRadius,
                std::vector<int8_t>& rArr,
                const int& rVal,
                const int& rWidth,
                const int& rHeight,
                const int8_t& occupancyThreshold = 90,
                const bool& ignoreObstacles=false) {
    int index;
    int dx, dy;
    int r_squared = rRadius * rRadius;
    for(int y = rY - rRadius; y <= rY + rRadius; ++y) {
        for(int x = rX - rRadius; x <= rX + rRadius; ++x) {
            if(x >= rWidth || y >= rHeight ||x < 0 || y < 0) continue;
            
            index = y * rWidth + x;

            // I use this condition to check if I should inflate or not obsacles
            // this is useful to clear the space for planning near the robot's pose
            if(ignoreObstacles == true && rArr[index] >= occupancyThreshold) continue;
            
            // check if y and x are inside the circle
            // or if they satisfy the circle equation
            dx = (x - rX);
            dy = (y - rY);
            dx *= dx;
            dy *= dy;
            if(dx + dy <= r_squared) rArr[index] = rVal;
        }
    }
}

#endif
