#ifndef _MANAGER_
#define _MANAGER_

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "quad_control/PlanRequest.h"
#include "quad_control/Trajectory.h"

/*
 * General manager that coordinates the operations. In detail:
 *  - sends appropriate requests for motion planning
 *  - converts the trajectory in a more convenient format for plotting
 *  - converts the 3D Gazebo scene into 3D octomap and 2D occupancy grid
 */
class Manager{

public:
    Manager();

    /* Request motion planning */
    void requestPlanning();

    /* Main loop */
    void run();

private:
    /* Save the occupancy grid read from the appropriate topic */
    void _map_cb(nav_msgs::OccupancyGridConstPtr data);

private:
    ros::NodeHandle _nh;
    ros::Subscriber _mapSub;
    ros::Publisher _planPub, _trajPub;

    double _rate;
    bool _mapReady;                     // Map status
    bool _trajPublishing;               // Trajectory status
    bool _toPlan, _planned;             // Planning status
    int _currTrajPoint;                 // Current trajectory point
    int _trajPerc;                      // Percentage of published trajectory

    quad_control::PlanRequest _req;     // Planning request
    quad_control::Trajectory _traj;     // Planned trajectory

};

#endif
