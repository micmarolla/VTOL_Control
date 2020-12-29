#ifndef _H_CONTROLLER_
#define _H_CONTROLLER_

#include "ros/ros.h"
#include <Eigen/Dense>
#include "quad_control/UAVPose.h"
#include "geometry_msgs/Accel.h"
#include "nav_msgs/Odometry.h"
#include "quad_control/Trajectory.h"
#include "LP2Filter.h"

#define GRAVITY 9.81

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



// Members and functions used by the PController subclass:
protected:
    ros::NodeHandle _nh;
    double _rate;
    double _m;                              // Mass
    Matrix3d _Ib;                           // Inertia
    Matrix3d _Rb;                           // Body rotation matrix wrt worldNED
    double _uT;                             // Total thrust (cmd)
    Vector3d _tau;                          // Torques (cmd)
    Matrix3d _Q, _QT, _Q_inv, _Q_dot;       // Q matrix
    Matrix3d _C;                            // Coriolis matrix
    Vector3d _p_d;                          // Linear velocity
    Vector3d _eta, _eta_d;
    Vector3d _deta_d, _deta_dd;
    Matrix<double,6,1> _e_eta;
    Matrix<double,6,1> _e_p;
    Vector3d _mud;
    geometry_msgs::Accel _da;


    /*
     * Compute Q matrix. It is stored in the class members, as well as
     * Q transposed, Q inversed and Q dot.
     */
    void _computeQ(Vector3d eta);

    /*
     * Compute Coriolis matrix. Q matrix must have been computed before to
     * call this.
     */
    void _computeC(Vector3d omega);

    /*
     * Execute hierarchical controller outer loop: compute uT and desired eta.
     */
    void _outerLoop();

    /*
     * Execute hierarchical controller inner loop: compute tau.
     */
    void _innerLoop();

    virtual void _coreLoop();

    virtual void _computeMu();

    virtual void _computeTau();



private:
    ros::Subscriber _trajSub;               // Trajectory
    ros::Subscriber _odomSub;               // Odometry
    ros::Publisher _pub;                    // Command wrench

    Matrix<double,3,6> _Kp, _Ke;            // Proportional gains
    Matrix<double,3,6> _Kpi, _Kei;          // Integral gains

    Trajectory _traj;                       // Trajectory
    bool _trajReady;                        // True if trajectory is ready

    ros::Time _startTime;                   // Time of trajectory starting
    bool _started, _completed;              // Flag about trajectory status

    UAVPose _dp;                            // Desired pos
    geometry_msgs::Accel _dv;               // Desired vel and acc

    nav_msgs::Odometry _odom;
    bool _odomReady;                        // True if odometry is ready

    Vector3d _p;
    Vector3d _omega;

    Matrix<double,6,1> _epInt, _eoInt;      // Integral
    Vector3d _deta;                         // Desired eta

    LP2Filter<Vector2d> _filter;            // Low-pass 2nd order filter
    int _filterSteps;               // Number of filtering steps for each value


    /*
     * Retrieve the current point of the trajectory, on the basis of trajectory
     * starting time and Controller rate.
     */
    void _getCurrentTrajPoint();

};

#endif
