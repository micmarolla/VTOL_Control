#ifndef _H_CONTROLLER_
#define _H_CONTROLLER_

#include "ros/ros.h"
#include <Eigen/Dense>
#include "quad_control/UAVPose.h"
#include "geometry_msgs/Accel.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/Trajectory.h"
#include "LP2Filter.h"

using namespace Eigen;
using namespace quad_control;

/*
 * This is a hierarchical controller for the VTOL UAV. Starting from desired
 * positions, velocities and accelerations, computes control wrenches.
 */
class Controller{

public:
    Controller();

    // Main loop
    void run();

    // Trajectory received callback
    void trajectoryReceived(TrajectoryPtr traj);

    // Odometry received callback
    void odomReceived(nav_msgs::OdometryPtr odom);


private:
    ros::NodeHandle _nh;
    ros::Subscriber _trajSub;               // Trajectory
    ros::Subscriber _odomSub;               // Odometry
    ros::Publisher _pub;                    // Command wrench

    double _m;                              // Mass
    Matrix3d _Ib;                           // Inertia
    Matrix<double,3,6> _Kp, _Ke;            // Proportional gains
    Matrix<double,3,6> _Kpi, _Kei;          // Integral gains
    double _rate;

    Trajectory _traj;                       // Trajectory
    bool _trajReady;                        // True if trajectory is ready

    ros::Time _startTime;                   // Time of trajectory starting
    bool _started, _completed;              // Flag about trajectory status

    UAVPose _dp;                            // Desired pos
    geometry_msgs::Accel _dv, _da;          // Desired vel and acc

    nav_msgs::Odometry _odom;
    bool _odomReady;                        // True if odometry is ready
    Matrix3d _Rb;                           // Body rotation matrix wrt worldNED

    Matrix<double,6,1> _epInt, _eoInt;      // Integral
    Vector3d _deta;                         // Desired eta
    Vector3d _mud;

    double _uT;
    Vector3d _tau;

    LP2Filter<Vector2d> _filter;            // Low-pass 2nd order filter
    int _filterSteps;               // Number of filtering steps for each value


    /*
     * Retrieve the current point of the trajectory, on the basis of trajectory
     * starting time and Controller rate.
     */
    void _getCurrentTrajPoint();

    /*
     * Execute hierarchical controller inner loop: compute tau.
     */
    void _innerLoop();

    /*
     * Execute hierarchical controller outer loop: compute uT and desired eta.
     */
    void _outerLoop();

};

#endif
