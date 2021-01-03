#include "NavigationFunc.h"
#include <queue>
#include <iostream>

/*
 * If map[i] > NAVFUNC_TRESH, it is assumed that there's an obstacle in that
 * cell. Map values goes from 0 to 100: for map retrieved from the octomap,
 * map only contains value 0 or 100, and nothing in between.
 */
#define NAVFUNC_TRESH  50

using namespace std;

NavigationFunc::NavigationFunc(){
    _map = 0;
    _nav = 0;
    _ready = false;
    _w = _h = 0;
}

NavigationFunc::~NavigationFunc(){
    delete _nav;
    if(_copied)
        delete _map;
}

void NavigationFunc::setMap(nav_msgs::OccupancyGrid &grid){
    this->setMap(&grid.data[0], grid.info.width, grid.info.height, true);
}

void NavigationFunc::setMap(int8_t *map, int w, int h, bool copyMap){
    int size = w*h;

    if(_copied)
        delete this->_map;

    // Retrieve the new map
    if(copyMap){
        this->_map = new int8_t[size];
        for (int i=0; i<size; ++i)
            this->_map[i] = map[i];
    }else
        this->_map = map;
    _copied = copyMap;

    // Clear navigation map
    delete this->_nav;
    this->_nav = new int[size];
    for (int i=0; i<size; ++i)
        this->_nav[i] = -1;

    this->_w = w;
    this->_h = h;

    this->_ready = true;
}


void NavigationFunc::scan(int goalX, int goalY){
    int index = 0, value = 0;
    int x = goalX, y = goalY;
    int valCounter = 0, nextValCounter = 0, cellCounter = 0;
    int tempIndex = 0;
    std::queue<int> cells;  // upcoming cells to visit

    // Set goal value to zero
    index = x * _w + y;
    this->_nav[index] = 0;
    ++cellCounter;
    ++valCounter;
    cells.push(index);

    while(cellCounter < _w * _h){
        // This should never happen!
        if(cells.empty()){
            ROS_FATAL_ERROR("Error in scanning navigation function.");
            return;
        }
        index = cells.front();
        cells.pop();
        y = index % _w;
        x = index / _w;

        // Get current value
        --valCounter;
        if(valCounter <= 0){
            ++value;
            valCounter = nextValCounter;
            nextValCounter = 0;
        }

        /* Retrieve neighbours */
        // Up
        if(x < _h - 1){
            tempIndex = index + _w;
            if(_nav[tempIndex] == -1){
                cells.push(tempIndex);
                ++cellCounter;
                if(_map[tempIndex] < NAVFUNC_TRESH){
                    _nav[tempIndex] = value;
                    ++nextValCounter;
                }
            }
        }

        // Left
        if(y > 0){
            tempIndex = index - 1;
            if(_nav[tempIndex] == -1){
                cells.push(tempIndex);
                ++cellCounter;
                if(_map[tempIndex] < NAVFUNC_TRESH){
                    _nav[tempIndex] = value;
                    ++nextValCounter;
                }
            }
        }

        // Right
        if(y < _w - 1){
            tempIndex = index + 1;
            if(_nav[tempIndex] == -1){
                cells.push(tempIndex);
                ++cellCounter;
                if(_map[tempIndex] < NAVFUNC_TRESH){
                    _nav[tempIndex] = value;
                    ++nextValCounter;
                }
            }
        }

        // Down
        if(x > 0){
            tempIndex = index - _w;
            if(_nav[tempIndex] == -1){
                cells.push(tempIndex);
                ++cellCounter;
                if(_map[tempIndex] < NAVFUNC_TRESH){
                    _nav[tempIndex] = value;
                    ++nextValCounter;
                }
            }
        }
    }

    _scanned = true;
}
