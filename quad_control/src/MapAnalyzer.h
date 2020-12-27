#ifndef _MAP_ANALYZER_
#define _MAP_ANALYZER_

#include <cstdint>
#include <vector>
#include "nav_msgs/OccupancyGrid.h"

using namespace std;

/*
 * Chunk is a tree-like structure, where each node has three child (left, right,
 * down). Used to detect and store obstacles in the map. Since the map is
 * scanned from top-left corner to bottom-right corner, the 'up' node is not
 * needed, since it will always be the root of the corresponding subtree.
 */
typedef struct Chunk{
    int x=0, y=0;
    double dist2=0;
    Chunk *left=0, *right=0, *down=0;
} Chunk;


/*
 * MapAnalyzer is an object that analyzes a 2D occupancy grid map, detecting
 * chunk of pixels, and the pixel at the minimum distance from the robot.
 * N.B.: every distance is expressed in 'cells', not in meters.
 */
class MapAnalyzer{

public:
    MapAnalyzer();
    ~MapAnalyzer();

    /*
     * Set the environment map.
     */
    void setMap(nav_msgs::OccupancyGrid &grid);

    /*
     * Set the environment map.
     * Parameters:
     *  - map: int8_t map data. This is designed to work with
     *         nav_msgs/OccupancyGrid: the data are stored in the map vector
     *         considering the (0,0) element in the bottom-left corner.
     *  - w, h: map's width and height
     */
    void setMap(int8_t *map, int w, int h);

    /*
     * Return true if a map has been loaded, false else.
     */
    bool ready(){ return this->_mapReady; }

    /*
     * Return true if the map has been scanned, false else.
     */
    bool scanned(){ return this->_scanned; }

    /*
     * Scan the map searching for chunks of obstacles. The scan is performed
     * from top-left to bottom-right corner; thus, the top-left corner of each
     * obstacle is the root of the corresponding chunk tree.
     * The detected chunks are stored in the private member variable _chunks.
     */
    void scan();

    /*
     * Detect for each chunk the point with minimum distance with respect to
     * the robot. Return a vector of the minimum-distance points.
     * Parameters:
     *  - rx, ry: robot's coordinates in the grid, expressed in cells
     *            (not meters!)
     */
    vector<Chunk*> getObjAtMinDist(int rx, int ry);


private:
    vector<Chunk*> _chunks;     // Roots of detected chunks
    int8_t* _map;
    bool _mapReady;             // True if map has been loaded
    bool _scanned;              // True if map has been scanned
    int _w, _h;                 // Map's width and height
    bool *_visited;             // Bool map: 1 = visited, 0 = not visited

    /*
     * Starting from the given root chunk, fill the tree recursively.
     * Parameters:
     *  - root: the tree chunk root
     *  - index: the index of the cell corresponding to the root chunk.
     *          NB: the 0 element corresponds to the bottom-left corner.
     */
    void _fillTree(Chunk *root, int index);

    /*
     * Return the point inside the given chunk with the minimum distance with
     * respect to the robot.
     * Parameters:
     *  - chunk: the root of the chunk tree to consider
     *  - rx, ry: the robot's position, expressed in cells
     */
    Chunk* _computeChunkDist(Chunk *chunk, int rx, int ry);

    /*
     * Delete recursively the chunk tree.
     */
    void _deleteTree(Chunk *root);

};

#endif
