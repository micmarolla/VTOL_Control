#ifndef _H_CONTROLLER_
#define _H_CONTROLLER_

#include "ros/ros.h"

/*
 * This is a hierarchical controller for the VTOL UAV. Starting from desired
 * positions, velocities and accelerations, computes control wrenches.
 */
class Controller{

public:


private:
    ros::NodeHandle _nh;

};

#endif
