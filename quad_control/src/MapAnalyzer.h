/*
 * MapAnalyzer is an object that analize a 2D occupancy grid map, detecting
 * chunk of pixels, and the pixel at the minimum distance from the robot.
 * Chunk is a tree structure, where each node has three child (left, right,
 * down).
 */

#include <cstdint>
#include <vector>

using namespace std;

typedef struct Chunk{
    int x=0, y=0;
    double dist2=0;
    Chunk *left=0, *right=0, *down=0;
} Chunk;


class MapAnalyzer{

private:
    vector<Chunk*> _chunks;
    int8_t *_map;
    bool *_visited;

    void _fillTree(Chunk *root, int index, int w, int h);
    Chunk* _computeChunkDist(Chunk *chunk, int rx, int ry);

public:
    MapAnalyzer();
    ~MapAnalyzer();

    void analyze(int8_t *map, int w, int h);
    vector<Chunk*> getObjAtMinDist(int rx, int ry);
    
};
