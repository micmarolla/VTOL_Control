#ifndef _QUAD_CONTROL_COMMON_H
#define _QUAD_CONTROL_COMMON_H

/*
 * If map[i] > MAP_TRESHOLD, it is assumed that there's an obstacle in that
 * cell. Map values goes from 0 to 100: for map retrieved from the octomap,
 * map only contains value 0 or 100, and nothing in between.
 */
#define     MAP_TRESHOLD    50

#define     GRAVITY         9.81

#endif
