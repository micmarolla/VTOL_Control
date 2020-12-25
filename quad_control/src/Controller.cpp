#include "Controller.h"
#include <cmath>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include "geometry_msgs/Wrench.h"
#include "Utils.h"

#define GRAVITY 9.81

Controller::Controller() : _nh("~"){
    // Retrieve params. Default values are for AscTec Hummingbird UAV
    _m = _nh.param<double>("m", 0.68);
    _Ib = Eigen::Matrix3d::Identity();
    _Ib(0,0) = _nh.param<double>("Ibx", 0.007);
    _Ib(1,1) = _nh.param<double>("Iby", 0.007);
    _Ib(2,2) = _nh.param<double>("Ibz", 0.012);

    // TODO: Read Kp, Ke as matrix
    double kp = _nh.param<double>("kp", 10.0);
    double ke = _nh.param<double>("ke", 10.0);
    double kpv = _nh.param<double>("kpv", 5.0);
    double kev = _nh.param<double>("kev", 5.0);
    _Kp << kp*Matrix3d::Identity(), kpv*Matrix3d::Identity();
    _Ke << ke*Matrix3d::Identity(), kev*Matrix3d::Identity();

    _rate = _nh.param<double>("rate", 1000);

    _trajReady = false;
    _odomReady = false;
    _started = false;
    _completed = false;
    _hovering = false;

    double k1 = _nh.param<double>("k1", 100.0);
    double k2 = _nh.param<double>("k2", 100.0);
    _filter.initFilterStep(0.001, k1, k2, Vector2d::Zero(), Vector2d::Zero());

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

    // Get current rotation matrix of body frame w.r.t. worldNED frame
    tf::matrixTFToEigen(tf::Matrix3x3(tf::Quaternion(_odom.pose.pose.orientation.x,
        _odom.pose.pose.orientation.y, _odom.pose.pose.orientation.z,
        _odom.pose.pose.orientation.w)), _Rb);

    _odomReady = true;
}


void Controller::_getCurrentTrajPoint(){
    if(_completed)
        return;

    // Get current trajectory point
    ros::Duration elapsed = ros::Time::now() - _startTime;

    int steps = int(elapsed.toSec() / _traj.sampleTime);

    if(_traj.p.size() <= steps || _traj.v.size() <= steps || _traj.a.size() <= steps){
        ROS_INFO("Trajectory completed! Entering hovering");
        _completed = true;
        _hovering = true;
        return;
    }

    // Retrieve current trajectory point
    _dp = _traj.p[steps];
    _dv = _traj.v[steps];
    _da = _traj.a[steps];
}


void Controller::_outerLoop(){
    //ROS_INFO("Outer loop");

    /* Compute position and linear velocity error */
    geometry_msgs::Pose& pose = this->_odom.pose.pose;
    geometry_msgs::Twist& twist = this->_odom.twist.twist;

    // Position error
    Eigen::Vector3d ep (pose.position.x - _dp.position.x,
            pose.position.y - _dp.position.y,
            pose.position.z - _dp.position.z);
    //ROS_INFO_STREAM("    Position error: " << ep[0] << ", " << ep[1] << ", " << ep[2]); ////

    // Transform velocity from body frame to worldNED frame
    Eigen::Vector3d linVel (twist.linear.x, twist.linear.y, twist.linear.z);
    linVel = _Rb * linVel;

    // Linear velocity error
    Eigen::Vector3d epd (linVel.x() - _dv.linear.x,
            linVel.y() - _dv.linear.y,
            linVel.z() - _dv.linear.z);
    //ROS_INFO_STREAM("    Velocity error: " << epd[0] << ", " << epd[1] << ", " << epd[2]); ////


    // Compute mu_d
    Eigen::Matrix<double,6,1> e;
    e << ep, epd;

    _mud = -_Kp*e + Eigen::Vector3d(_da.linear.x, _da.linear.y, _da.linear.z);
    //ROS_INFO_STREAM("    mud: " << _mud[0] << ", " << _mud[1] << ", " << _mud[2]); ////


    // Compute outputs
    _uT = _m * sqrt(pow(_mud[0],2) + pow(_mud[1],2) + pow(_mud[2] - GRAVITY,2));
    //ROS_INFO_STREAM("    uT: " << _uT); ////////

    _deta[0] = asin(_m * (_mud[1]*cos(_dp.yaw) - _mud[0]*sin(_dp.yaw)) / _uT);
    _deta[1] = atan2(_mud[0]*cos(_dp.yaw) + _mud[1]*sin(_dp.yaw), _mud[2] - GRAVITY);
    _deta[2] = _dp.yaw;
    //ROS_INFO_STREAM("    des_eta: " << _deta[0] << ", " << _deta[1] << ", " << _deta[2]); //////
}


void Controller::_innerLoop(){
    //ROS_INFO("Inner loop");

    // Compute derivatives of orientation
    Eigen::Vector3d deta_d, deta_dd;
    this->_filter.filterStep(_deta.segment<2>(0));
    deta_d << _filter.lastFirst(), _dv.angular.z;
    deta_dd << _filter.lastSecond(), _da.angular.z;

    // Compute orientation errors
    geometry_msgs::Pose& pose = this->_odom.pose.pose;
    geometry_msgs::Twist& twist = this->_odom.twist.twist;
    /*double r,p,y;
    tf::Matrix3x3(tf::Quaternion(pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w)).getRPY(r,p,y);*/
    Eigen::Vector3d eta = _Rb.eulerAngles(0,1,2);
    Eigen::Vector3d eo = eta - _deta;

    //ROS_INFO_STREAM("    eta: " << eta[0] << ", " << eta[1] << ", " << eta[2]); //////
    //ROS_INFO_STREAM("    des_eta: " << _deta[0] << ", " << _deta[1] << ", " << _deta[2]); //////
    //ROS_INFO_STREAM("    eo: " << eo[0] << ", " << eo[1] << ", " << eo[2]); //////

    // Transform angular velocity from body frame to worldNED frame
    Eigen::Vector3d angVel (twist.angular.x, twist.angular.y, twist.angular.z);
    angVel = _Rb * angVel;

    // Compute angular velocity error
    Eigen::Vector3d eod = angVel - deta_d;
    //ROS_INFO_STREAM("    des_eta_d: " << deta_d[0] << ", " << deta_d[1] << ", " << deta_d[2]); //////
    //ROS_INFO_STREAM("    angVel: " << angVel[0] << ", " << angVel[1] << ", " << angVel[2]); //////
    //ROS_INFO_STREAM("    eod: " << eod[0] << ", " << eod[1] << ", " << eod[2]); //////
    //ROS_INFO_STREAM("    des_eta_dd: " << deta_dd[0] << ", " << deta_dd[1] << ", " << deta_dd[2]); //////

    // Compute tau_tilde
    Eigen::Matrix<double,6,1> e;
    e << eo, eod;

    // Compute Coriolis matrix
    Eigen::Matrix3d Q, QT, Q_dot, C;
    Q << 1,     0,              -sin(eta[0]),
         0,     cos(eta[0]),    cos(eta[1])*sin(eta[0]),
         0,     -sin(eta[0]),   cos(eta[1])*cos(eta[0]);
    QT = Q.transpose();
    Q_dot << 0, 0,              -cos(eta[0]),
             0, -sin(eta[0]),   -sin(eta[1])*sin(eta[0])+cos(eta[1])*cos(eta[0]),
             0, -cos(eta[0]),   -sin(eta[1])*cos(eta[0])-cos(eta[1])*sin(eta[0]);
    C = QT * skew(Q*angVel) * _Ib * Q  +  QT * _Ib * Q_dot;

    // Compute tau
    _tau = _Ib*Q*(-_Ke*e + deta_dd) + Q.inverse().transpose() * C * angVel;
    //ROS_INFO_STREAM("    tau: " << _tau[0] << ", " << _tau[1] << ", " << _tau[2]); //////
}


void Controller::run(){
    ROS_INFO("VTOL UAV Controller started.");

    ros::Rate r(_rate);
    geometry_msgs::Wrench cmd;

    // These forces are not considered by the quadrotor.
    cmd.force.x = cmd.force.y = 0;

    while(ros::ok()){
        if(_trajReady && _odomReady){
            if(!_started){
                _startTime = ros::Time::now();
                _started = true;
                ROS_INFO("Starting trajectory");
            }

            _getCurrentTrajPoint();
            _outerLoop();       // compute uT
            _innerLoop();       // compute tau

            // Build command msg and publish it
            cmd.force.z = _uT;
            cmd.torque.x = 0;//_tau[0];
            cmd.torque.y = 0;//_tau[1];
            cmd.torque.z = 0;//_tau[2];

            _pub.publish(cmd);

            //ROS_INFO("================================================");
        }

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
