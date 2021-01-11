#include "MapAnalyzer.h"

#include <cmath>
#include <ros/console.h>
#include "common.h"

MapAnalyzer::MapAnalyzer(){
    _map = 0;
    _mapReady = false;
    _scanned = false;
    _w = _h = 0;
    _visited = 0;
}

MapAnalyzer::~MapAnalyzer(){
    for (auto chunk : this->_chunks)
        _deleteTree(chunk);
    delete this->_visited;
    if(_copied)
        delete this->_map;
}

void MapAnalyzer::_deleteTree(Chunk *root){
    if(root->left)  _deleteTree(root->left);
    if(root->right) _deleteTree(root->right);
    if(root->down)  _deleteTree(root->down);
    delete root;
}


void MapAnalyzer::_fillTree(Chunk *root, int index){
    if(!this->_map || !this->_visited)
        return;

    this->_visited[index] = true;
    if(this->_map[index] < MAP_TRESHOLD)
        return;

    // Left node
    if(root->y > 0){
        int k = index - 1;
        if (!this->_visited[k] && this->_map[k] > MAP_TRESHOLD){
            root->left = new Chunk;
            root->left->x = root->x;
            root->left->y = root->y-1;
            _fillTree(root->left, k);
        }
    }

    // Right node
    if(root->y < this->_w - 1){
        int k = index + 1;
        if (!this->_visited[k] && this->_map[k] > MAP_TRESHOLD){
            root->right = new Chunk;
            root->right->x = root->x;
            root->right->y = root->y+1;
            _fillTree(root->right, k);
        }
    }

    // Bottom node
    if(root->x > 0){
        int k = index - this->_w;
        if (!this->_visited[k] && this->_map[k] > MAP_TRESHOLD){
            root->down = new Chunk;
            root->down->x = root->x-1;
            root->down->y = root->y;
            _fillTree(root->down, k);
        }
    }
}


void MapAnalyzer::setMap(nav_msgs::OccupancyGrid &grid){
    this->setMap(&grid.data[0], grid.info.width, grid.info.height, true);
}


void MapAnalyzer::setMap(int8_t *map, int w, int h, bool copyMap){
    if(_copied)
        delete this->_map;

    int size = w*h;
    if(copyMap){
        this->_map = new int8_t[size];
        for (int i=0; i<size; ++i)
            this->_map[i] = map[i];
    }else
        this->_map = map;
    _copied = copyMap;

    // Clear chunks
    for (auto chunk : this->_chunks)
        _deleteTree(chunk);
    this->_chunks.clear();

    // Clear visited
    delete this->_visited;
    this->_visited = new bool[size];
    for (int i=0; i<size; ++i)
        this->_visited[i] = false;

    this->_w = w;
    this->_h = h;

    this->_mapReady = true;
}


void MapAnalyzer::scan(){
    ROS_INFO("Scanning map...");

    // Scan the map
    for (int r=_h-1; r>=0; --r){
        for (int c=0; c < _w; ++c){
            int i = r * _w + c;

            if (this->_map[i] < MAP_TRESHOLD || this->_visited[i]){
                this->_visited[i] = true;
                continue;
            }

            // Fill tree
            Chunk *obst = new Chunk;
            // Coords are inverted wrt the usual way, since the frame is NED
            obst->y = i % _w;
            obst->x = i / _w;
            this->_fillTree(obst, i);

            this->_chunks.push_back(obst);
        }
    }

    _scanned = true;
    ROS_INFO("Map scanned.");
}


int8_t MapAnalyzer::cellValue(int x, int y){
    if (!_scanned)
        return -1;
    return _map[x*_w + y];
}


Chunk* MapAnalyzer::_computeChunkDist(Chunk *chunk, int rx, int ry){
    if (chunk == 0)
        return 0;

    Chunk *best = chunk;
    Chunk *temp = 0;
    chunk->dist2 = pow(chunk->x - rx, 2) + pow(chunk->y - ry, 2);

    if(chunk->left){
        temp = _computeChunkDist(chunk->left, rx, ry);
        if(temp->dist2 < best->dist2)
            best = temp;
    }

    if(chunk->right){
        temp = _computeChunkDist(chunk->right, rx, ry);
        if(temp->dist2 < best->dist2)
            best = temp;
    }

    if(chunk->down){
        temp = _computeChunkDist(chunk->down, rx, ry);
        if(temp->dist2 < best->dist2)
            best = temp;
    }

    return best;
}


vector<Chunk*> MapAnalyzer::getObjAtMinDist(int rx, int ry){
    vector<Chunk*> mins;

    // Compute distances
    for (auto chunk : this->_chunks)
        mins.push_back(_computeChunkDist(chunk, rx, ry));

    return mins;
}

int8_t* MapAnalyzer::generateSubmap(int rx, int ry, int w, int h){
    int8_t* submap = new int8_t[w*h];

    int subIndex = 0, mapIndex = 0;
    int mapX = (rx - ceil((w-1)/2));
    int mapY = (ry - ceil((h-1)/2));

    while (subIndex < w*h){
        mapIndex =  mapX * this->_w + mapY;
        submap[subIndex++] = _map[mapIndex];

        if(mapY < w - 1)
            ++mapY;
        else{
            ++mapX;
            mapY = 0;
        }

    }

    return submap;
}
