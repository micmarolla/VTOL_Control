#ifndef _H_CONTROLLER_
#define _H_CONTROLLER_

#include <ros/ros.h>
#include <geometry_msgs/Accel.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Dense>

#include "quad_control/UAVPose.h"
#include "quad_control/Trajectory.h"
#include "LP2Filter.h"

using namespace Eigen;
using namespace quad_control;

typedef Matrix<double,6,1> Vector6d;

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
    Vector3d _mud;                          // Desired mu (used to compute uT)
    Vector3d _tau;                          // Torques (cmd)

    Matrix3d _Q, _QT, _Q_inv, _Q_dot;       // Q matrix
    Matrix3d _C;                            // Coriolis matrix

    Vector3d _p_d;                          // Linear velocity
    Vector3d _eta, _eta_d;                  // Orientation and its derivative
    geometry_msgs::Accel _da;               // Desired acceleration
    Vector3d _deta_d, _deta_dd;             // Desired orientation and its der.

    Vector6d _e_eta;                        // Orientation and der. error
    Vector6d _e_p;                          // Position and lin vel error

    Vector3d _epInt, _eoInt;                // Integral
    Matrix3d _Kpi, _Kei;                    // Integral gains


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

    /* Controller core loop. Can be redefined by subclasses. */
    virtual void _coreLoop();

    /* Compute desired mu. Can be redefined by subclasses. */
    virtual void _computeMu();

    /* Compute tau (control wrench). Can be redefined by subclasses. */
    virtual void _computeTau();



private:
    ros::Subscriber _trajSub;               // Trajectory
    ros::Subscriber _odomSub;               // Odometry
    ros::Publisher _pub;                    // Command wrench
    ros::Publisher _pointPub;               // Current trajectory point
    ros::Publisher _trajPub;                // Current desired pose

    Matrix<double,3,6> _Kp, _Ke;            // Proportional gains

    Trajectory _traj;                       // Trajectory
    bool _trajReady;                        // True if trajectory is ready
    int _trajStep;                          // Current trajectory step
    int _remainingSteps;                    // ..for the current traj point

    bool _waiting, _tracking, _landing;     // Trajectory status
    bool _willLand;                         // True for landing, false for hovering

    UAVPose _dp;                            // Desired pos
    geometry_msgs::Accel _dv;               // Desired linear velocity
    Vector3d _deta;                         // Desired orientation

    nav_msgs::Odometry _odom;               // Odometry
    bool _odomReady;                        // True if odometry is ready
    Vector3d _p;                            // Current position
    Vector3d _omega;                        // Current angular velocity

    LP2Filter<Vector2d> _filter;            // Low-pass 2nd order filter
    int _filterSteps;           // Number of filtering steps for each new value

    bool _plotTrajectory;       // Plot the current desired pos


    /*
     * Retrieve the current point of the trajectory, on the basis of trajectory
     * starting time and Controller rate.
     */
    void _getCurrentTrajPoint();

};

#endif
