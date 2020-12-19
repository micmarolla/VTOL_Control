#include "Controller.h"
#include <cmath>
#include <tf/LinearMath/Matrix3x3.h>
#include "geometry_msgs/Wrench.h"

#define GRAVITY 9.81

Controller::Controller() : _nh("~"){
    // Retrieve params. Default values are for AscTec Hummingbird UAV
    _m = _nh.param<double>("m", 0.68);
    _Ib = Eigen::Matrix3d::Identity();
    _Ib(0,0) = _nh.param<double>("Ibx", 0.007);
    _Ib(1,1) = _nh.param<double>("Iby", 0.007);
    _Ib(2,2) = _nh.param<double>("Ibz", 0.012);

    // Read Kp, Ke as matrix
    
    _rate = _nh.param<double>("rate", 0.001);

    _trajReady = false;
    _odomReady = false;

    _trajSub = _nh.subscribe("/trajectory", 0, &Controller::trajectoryReceived, this);
    _odomSub = _nh.subscribe("/hummingbird/ground_truth/odometryNED", 0, &Controller::odomReceived, this);
    _pub = _nh.advertise<geometry_msgs::Wrench>("/hummingbird/command/wrenchNED", 0);
}


void Controller::trajectoryReceived(quad_control::TrajectoryPtr traj){
    _traj = *traj;
    _trajReady = true;
}

void Controller::odomReceived(nav_msgs::OdometryPtr odom){
    _odom = *odom;
    _odomReady = true;
}


void Controller::_getCurrentTrajPoint(){
    // Get current trajectory point
    // Transform lin and ang velocity from body frame to world NED frame
    //  .. compute Rb using tf::Matrix from quaternion in desired pose, then
    //      convert with tf::matrixTFToEigen (in "tf/tf_eigen.h")
}

void Controller::_outerLoop(){
    /* Compute position and linear velocity error */
    geometry_msgs::Pose& pose = this->_odom.pose.pose;
    geometry_msgs::Twist& twist = this->_odom.twist.twist;

    // Position error
    Eigen::Vector3d _ep (pose.position.x - _dp.position.x,
            pose.position.y - _dp.position.y,
            pose.position.z - _dp.position.z);

    // Linear velocity error
    Eigen::Vector3d _epd (twist.linear.x - _dv.linear.x,
            twist.linear.y - _dv.linear.y,
            twist.linear.z - _dv.linear.z);


    /* Compute mu_d */
    Eigen::Matrix<double,6,1> e;
    e << _ep[0], _ep[1], _ep[2], _epd[0], _epd[1], _epd[2];

    _mud = -_Kp*e + Eigen::Vector3d(_da.linear.x, _da.linear.y, _da.linear.z);
    
    
    /* Compute outputs */
    _uT = _m * sqrt(pow(_mud[0],2) + pow(_mud[1],2) + pow(_mud[2] - GRAVITY,2));

    tf::Matrix3x3(tf::Quaternion(_dp.orientation.x, _dp.orientation.y,
        _dp.orientation.z, _dp.orientation.w)).getRPY(_dRoll,_dPitch,_dYaw);

    _dRoll = asin(_m * (_mud[1]*cos(_dYaw) - _mud[0]*sin(_dYaw)) / _uT);
    _dPitch = atan2(_mud[0]*cos(_dYaw) + _mud[1]*sin(_dYaw), _mud[2] - GRAVITY);
}


void Controller::_innerLoop(){
    // Use 2nd order low-pass filter to obtain derivatives of orientation
    // Compute orientation errors
    // Compute tau_tilde
    // Compute tau_b
}


void Controller::run(){
    ROS_INFO("VTOL UAV Controller started.");

    ros::Rate r(_rate);
    geometry_msgs::Wrench cmd;

    // These forces are not considered by the quadrotor.
    cmd.force.x = cmd.force.y = 0;

    while(ros::ok()){
        _getCurrentTrajPoint();
        _outerLoop();       // compute uT
        _innerLoop();       // compute tau

        // Build command msg and publish it
        cmd.force.z = _uT;
        cmd.torque.x = _tau[0];
        cmd.torque.y = _tau[1];
        cmd.torque.z = _tau[2];
        
        _pub.publish(cmd);

        r.sleep();
        ros::spinOnce();
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "hier_controller");
    Controller ctrl;
    ctrl.run();
    return 0;
}
