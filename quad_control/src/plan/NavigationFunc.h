#ifndef _NAVIGATION_FUNC
#define _NAVIGATION_FUNC

#include <cstdint>
#include <queue>
#include <nav_msgs/OccupancyGrid.h>

/*
 * This object generates a navigation function for a given map.
 * It can be used as a stand-alone planning method, or as a solution to the
 * local minima problem for artificial potential planning.
 */
class NavigationFunc{

public:
    NavigationFunc();
    ~NavigationFunc();

    /*
     * Set the environment map.
     * If res = 0, the same resolution as grid is used; otherwise, the
     * resolution nearest to res is used such that the ratio between res and
     * grid resolution is an integer.
     */
    void setMap(nav_msgs::OccupancyGrid &grid, float res=0);

    /*
     * Set the environment map.
     * Parameters:
     *  - map: int8_t map data. This is designed to work with
     *         nav_msgs/OccupancyGrid: the data are stored in the map vector
     *         considering the (0,0) element in the bottom-left corner.
     *  - w, h: map's width and height
     *  - ratio: ratio between navigation function resolution and map resolution
     *  - copyMap: if true, the map data is copied inside this object;
     *      otherwise, the content is just pointed, and memory management is
     *      not handled by this object.
     */
    void setMap(int8_t *map, int w, int h, int ratio=1, bool copyMap=false);

    /*
     * Return true if a map has been loaded, false else.
     */
    bool ready(){ return this->_ready; }

    /*
     * Return true if the map has been scanned, false else.
     */
    bool scanned(){ return this->_scanned; }

    /*
     * Return true if the map has been partially scanned, i.e., scanned only
     * to reach goal from one specific position; false else.
     */
    bool partiallyScanned(){ return this->_partiallyScanned; }

    /*
     * Scan the map, creating the navigation map.
     * ...
     */
    const int* scan(int goalX, int goalY, int eta, int rx=-1, int ry=-1);

    std::queue<int>* getPath(int rx=-1, int ry=-1);

    /*
     * Return true if the cell (x,y), or any neighbour in the given radius
     * is an obstacle.
     * In detail, if any of the cell inside the square of center (x,y) and
     * dimension eta is an obstacle, the cell itself is treated as an obstacle.
     * Parameters:
     *  - x, y: cell coordinates
     *  - eta: obstacle's isotropic expansion radius.
     */
    bool isObstacle(int x, int y, int eta);

    /*
     * Return true if the given cell, or any neighbour in the given radius
     * is an obstacle.
     * In detail, if any of the cell inside the square of center the given cell
     * and dimension eta is an obstacle, the cell itself is treated as an obstacle.
     * Parameters:
     *  - index: cell index
     *  - eta: obstacle's isotropic expansion radius.
     */
    bool isObstacle(int index, int eta);

private:
    /*
     * Check if the given cell is an obstacle (or near an obstacle). This is the
     * actual function called by isObstacle(). Check its documentation for
     * further details.
     */
    bool _isObstacle(int x, int y, int index, int eta);


private:
    int8_t* _map;
    int* _nav;                          // Navigation function
    int _w, _h;                         // Map's width and height
    int _robot;                         // Current robot index
    bool _copied;               // If true, map memory is handled by this obj
    bool _ready;                        // Status flags
    bool _scanned, _partiallyScanned;
    std::queue<int> _path;              // Last generated path

};

#endif
