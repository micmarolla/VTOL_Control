#include "MapAnalyzer.h"
#include <cmath>
#include <sstream>
#include "ros/console.h"

MapAnalyzer::MapAnalyzer(){
    this->_map = 0;
    this->_visited = 0;
    this->_resolution = 1;
}


MapAnalyzer::~MapAnalyzer(){
    for (auto chunk : this->_chunks)
        delete chunk;
}


void MapAnalyzer::_fillTree(Chunk *root, int index, int w, int h){
    if(!this->_map || !this->_visited)
        return;
    
    this->_visited[index] = true;
    if(this->_map[index] < 50)
        return;

    // Left node
    if(root->x > 0){
        int k = index - 1;
        if (!this->_visited[k] && this->_map[k]>50){
            root->left = new Chunk;
            root->left->x = root->x-1;
            root->left->y = root->y;
            _fillTree(root->left, k, w, h);
        }
    }

    // Right node
    if(root->x < w-1){
        int k = index + 1;
        if (!this->_visited[k] && this->_map[k]>50){
            root->right = new Chunk;
            root->right->x = root->x+1;
            root->right->y = root->y;
            _fillTree(root->right, k, w, h);
        }
    }

    // Bottom node
    if(root->y > 0){
        int k = index - w;
        if (!this->_visited[k] && this->_map[k]>50){
            root->down = new Chunk;
            root->down->x = root->x;
            root->down->y = root->y-1;
            _fillTree(root->down, k, w, h);
        }
    }
}


void MapAnalyzer::analyze(nav_msgs::OccupancyGrid &grid){
    this->analyze(&grid.data[0], grid.info.width, grid.info.height, grid.info.resolution);
}


void MapAnalyzer::analyze(int8_t *map, int w, int h, double resolution){
    this->_map = map;
    this->_resolution = resolution;

    for (auto chunk : this->_chunks)
        delete chunk;
    this->_chunks.clear();

    int size = w*h;

    delete this->_visited;
    this->_visited = new bool[size];
    for (int i=0; i<size; ++i)
        this->_visited[i] = false;

    // Scan the map
    for (int r=h-1; r>=0; --r){
        for (int c=0; c < w; ++c){
            int i = r*w + c;
            if (this->_map[i] < 50 || this->_visited[i]){
                this->_visited[i] = true;
                continue;
            }
    
            // Fill tree
            Chunk *obst = new Chunk;
            obst->x = i % w;
            obst->y = i / w;
            this->_fillTree(obst, i, w, h);
            
            this->_chunks.push_back(obst);
        }
    }
}


Chunk* MapAnalyzer::_computeChunkDist(Chunk *chunk, int rx, int ry){
    if (chunk == 0)
        return 0;

    Chunk *best = chunk;
    Chunk *temp = 0;
    chunk->dist2 = pow(chunk->x - rx, 2) + pow(chunk->y - ry, 2); // [cell^2]
    chunk->dist2 *= pow(_resolution, 2);    // Convert in meters^2
    
    if(chunk->left){
        temp = _computeChunkDist(chunk->left, rx, ry);
        if(temp->dist2 < best->dist2)
            best = temp;
    }if(chunk->right){
        temp = _computeChunkDist(chunk->right, rx, ry);
        if(temp->dist2 < best->dist2)
            best = temp;
    }if(chunk->down){
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

