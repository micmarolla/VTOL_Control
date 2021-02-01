#include "NavigationFunc.h"
#include <ros/console.h>
#include "common.h"

NavigationFunc::NavigationFunc(){
    _map = 0;
    _nav = 0;
    _ready = false;
    _scanned = false;
    _partiallyScanned = false;
    _w = _h = 0;
    _robot = -1;
}

NavigationFunc::~NavigationFunc(){
    delete _nav;
    if(_copied)
        delete _map;
}


void NavigationFunc::setMap(nav_msgs::OccupancyGrid &grid, float res){
    this->setMap(&grid.data[0], grid.info.width, grid.info.height,
        ceil(res / grid.info.resolution), true);
}


void NavigationFunc::setMap(int8_t *map, int _w, int _h, int ratio, bool copyMap){
    if(_copied)
        delete this->_map;

    int w = _w / ratio;
    int h = _h / ratio;
    int size = w * h;

    this->_map = new int8_t[size];

    // Build map with the given resolution ratio
    int x = 0, y = 0;
    int8_t max = 0;
    for (int i=0; i < size; ++i){
        x = i / w;
        y = i % w;
        max = 0;

        for(int xx = x*ratio; xx < (x+1)*ratio && xx < _h; ++xx)
            for(int yy = y*ratio; yy < (y+1)*ratio  && yy < _w; ++yy)
                if (map[xx*_w+yy] > max)
                    max = map[xx*_w+yy];
        this->_map[i] = max;
    }

    _copied = copyMap;

    // Clear navigation map
    delete this->_nav;
    this->_nav = new int[size];
    for (int i=0; i<size; ++i)
        this->_nav[i] = -1;

    this->_w = w;
    this->_h = h;

    this->_ready = true;
    this->_scanned = false;
    this->_partiallyScanned = false;
}


bool NavigationFunc::isObstacle(int x, int y, int eta){
    if(!_ready) return false;
    int index = x * this->_w + y;
    return _isObstacle(x, y, index, eta);
}

bool NavigationFunc::isObstacle(int index, int eta){
    if(!_ready) return false;
    int x = index / this->_w;
    int y = index % this->_w;
    return _isObstacle(x, y, index, eta);
}

bool NavigationFunc::_isObstacle(int x, int y, int index, int eta){
    if(!_ready)                             return false;
    if(_map[index] > MAP_THRESHOLD)         return true;

    int i;
    int firstRow = (x + eta < _h) ? (x + eta) : _h - 1;
    int lastRow  = (x - eta >= 0) ? (x - eta) : 0;
    int firstCol = (y - eta >= 0) ? (y - eta) : 0;
    int lastCol  = (y + eta < _w) ? (y + eta) : _w - 1;

    for (int r = firstRow; r >= lastRow; --r){
        for(int c = firstCol; c <= lastCol; ++c){
            i = r * this->_w + c;
            if(_map[i] > MAP_THRESHOLD)
                return true;
        }
    }
    return false;
}



const int* NavigationFunc::scan(int goalX, int goalY, int eta, int rx, int ry){
    int index = 0, value = 0;
    int x = goalX, y = goalY;
    int valCounter = 0, nextValCounter = 0;
    int tempIndex = 0;
    std::queue<int> cells;  // upcoming cells to visit

    // Clear the path
    std::queue<int>().swap(_path);

    this->_robot = rx * _w + ry;

    // Clear nav
    for(int i=0; i < _w*_h; ++i)
        _nav[i] = -1;

    // Set goal value to zero
    index = x * _w + y;
    this->_nav[index] = 0;
    ++valCounter;
    cells.push(index);

    if(_robot == index)
        return this->_nav;

    int count = 0;
    while(true){
        // Cannot scan other cells
        if(cells.empty()){
            ROS_ERROR("Unable to build the navigation function.");
            break;
        }

        index = cells.front();
        cells.pop();
        y = index % _w;
        x = index / _w;

        // Get current value
        --valCounter;
        if(valCounter <=  0){
            ++value;
            valCounter = nextValCounter;
            nextValCounter = 0;
        }

        /* Retrieve neighbours */
        // Up
        if(x < _h - 1){
            tempIndex = index + _w;
            if(_nav[tempIndex] == -1){
                if(!isObstacle(tempIndex, eta)){
                    cells.push(tempIndex);
                    _nav[tempIndex] = value;
                    ++nextValCounter;
                    if(_robot == tempIndex)
                        break;
                }
            }
        }

        // Left
        if(y > 0){
            tempIndex = index - 1;
            if(_nav[tempIndex] == -1){
                if(!isObstacle(tempIndex, eta)){
                    cells.push(tempIndex);
                    _nav[tempIndex] = value;
                    ++nextValCounter;
                    if(_robot == tempIndex)
                        break;
                }
            }
        }

        // Right
        if(y < _w - 1){
            tempIndex = index + 1;
            if(_nav[tempIndex] == -1){
                if(!isObstacle(tempIndex, eta)){
                    cells.push(tempIndex);
                    _nav[tempIndex] = value;
                    ++nextValCounter;
                    if(_robot == tempIndex)
                        break;
                }
            }
        }

        // Down
        if(x > 0){
            tempIndex = index - _w;
            if(_nav[tempIndex] == -1){
                if(!isObstacle(tempIndex, eta)){
                    cells.push(tempIndex);
                    _nav[tempIndex] = value;
                    ++nextValCounter;
                    if(_robot == tempIndex)
                        break;
                }
            }
        }

    }

    _scanned = true;
    if(rx != -1 && ry != -1)
        _partiallyScanned = true;

    return this->_nav;
}


std::queue<int>* NavigationFunc::getPath(int rx, int ry){
    if(rx == -1 && ry == -1 && !_path.empty())
        return &this->_path;

    // Clear the queue
    std::queue<int>().swap(_path);

    // Build the new path: steepest descent method
    int x = rx, y = ry;
    int curr = x * _w + y;
    if(rx == -1 && ry == -1)
        curr = _robot;
    int next = 0;
    while(_nav[curr] > 0){
        _path.push(curr);
        y = curr % _w;
        x = curr / _w;

        // Check up
        if(x < _h - 1){
            next = curr + _w;
            if(_nav[next] != -1 && _nav[next] <= _nav[curr]){
                curr = next;
                continue;
            }
        }

        // Check left
        if(y > 0){
            next = curr - 1;
            if(_nav[next] != -1 && _nav[next] <= _nav[curr]){
                curr = next;
                continue;
            }
        }

        // Check right
        if(y < _w){
            next = curr + 1;
            if(_nav[next] != -1 && _nav[next] <= _nav[curr]){
                curr = next;
                continue;
            }
        }

        // Check bottom
        if(x > 0){
            next = curr - _w;
            if(_nav[next] != -1 && _nav[next] <= _nav[curr]){
                curr = next;
                continue;
            }
        }
    }

    _path.push(curr);   // Add goal cell

    return &this->_path;

}
