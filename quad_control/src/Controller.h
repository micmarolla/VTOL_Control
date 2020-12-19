#ifndef _H_CONTROLLER_
#define _H_CONTROLLER_

#include "ros/ros.h"
#include <Eigen/Dense>
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Accel.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/Trajectory.h"

/*
 * This is a hierarchical controller for the VTOL UAV. Starting from desired
 * positions, velocities and accelerations, computes control wrenches.
 */
class Controller{

public:
    Controller();

    void run();

    void trajectoryReceived(quad_control::TrajectoryPtr traj);

    void odomReceived(nav_msgs::OdometryPtr odom);


private:
    ros::NodeHandle _nh;
    ros::Subscriber _trajSub;               // Trajectory
    ros::Subscriber _odomSub;               // Odometry
    ros::Publisher _pub;                    // Command wrench

    double _m;                              // Mass
    Eigen::Matrix3d _Ib;                    // Inertia
    Eigen::Matrix<double,3,6> _Kp, _Ke;     // Gains
    double _rate;
    
    quad_control::Trajectory _traj;         // Trajectory
    bool _trajReady;

    geometry_msgs::Pose _dp;                // Desired pos
    geometry_msgs::Accel _dv, _da;          // Desired vel and acc

    nav_msgs::Odometry _odom;
    bool _odomReady;

    double _dRoll, _dPitch, _dYaw;    
    Eigen::Vector3d _mud;
    double _uT;
    Eigen::Vector3d _tau;


    void _getCurrentTrajPoint();
    void _innerLoop();
    void _outerLoop();

};

#endif
