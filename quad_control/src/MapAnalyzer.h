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
 * MapAnalyzer is an object that analize a 2D occupancy grid map, detecting
 * chunk of pixels, and the pixel at the minimum distance from the robot.
 * N.B.: every distance is expressed in 'cells', not in meters.
 */
class MapAnalyzer{

public:
    MapAnalyzer();
    ~MapAnalyzer();

    /*
     * Scan the map searching for chunks of obstacles. The scan is performed
     * from top-left to bottom-right corner; thus, the top-left corner of each
     * obstacle is the root of the corresponding chunk tree.
     * The detected chunks are stored in the private member variable _chunks.
     */
    void analyze(nav_msgs::OccupancyGrid &grid);

    /*
     * Scan the map searching for chunks of obstacles. The scan is performed
     * from top-left to bottom-right corner; thus, the top-left corner of each
     * obstacle is the root of the corresponding chunk tree.
     * The detected chunks are stored in the private member variable _chunks.
     * Parameters:
     *  - map: int8_t map data. This is designed to work with
     *         nav_msgs/OccupancyGrid: the data are stored in the map vector
     *         considering the (0,0) element in the bottom-left corner.
     *  - w, h: map's width and height
     */
    void analyze(int8_t *map, int w, int h);

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
    int8_t *_map;
    int _w, _h;                 // Map's width and height
    bool *_visited;

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
