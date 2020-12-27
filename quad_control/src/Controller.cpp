#include "Controller.h"
#include <cmath>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include "geometry_msgs/Wrench.h"
#include "Utils.h"

#define GRAVITY 9.81

using namespace Eigen;

Controller::Controller() : _nh("~"){
    // Retrieve params. Default values are for AscTec Hummingbird UAV
    _m = _nh.param<double>("m", 0.68);
    _Ib = Matrix3d::Identity();
    _Ib(0,0) = _nh.param<double>("Ibx", 0.007);
    _Ib(1,1) = _nh.param<double>("Iby", 0.007);
    _Ib(2,2) = _nh.param<double>("Ibz", 0.012);

    // TODO: Read Kp, Ke as matrix
    double kp = _nh.param<double>("kp", 10.0);
    double kpv = _nh.param<double>("kpv", 10.0);
    double ke = _nh.param<double>("ke", 1.0);
    double kev = _nh.param<double>("kev", 1.0);
    _Kp << kp*Matrix3d::Identity(), kpv*Matrix3d::Identity();
    _Ke << ke*Matrix3d::Identity(), kev*Matrix3d::Identity();

    double kpi = _nh.param<double>("kpi", 0.0);
    double kei = _nh.param<double>("kei", 0.0);
    double kpiv = _nh.param<double>("kpiv", 0.0);
    double keiv = _nh.param<double>("keiv", 0.0);
    _Kpi << kpi*Matrix3d::Identity(), kpiv*Matrix3d::Identity();
    _Kei << kei*Matrix3d::Identity(), keiv*Matrix3d::Identity();

    _epInt << 0,0,0,0,0,0;
    _eoInt << 0,0,0,0,0,0;

    _rate = _nh.param<double>("rate", 1000.0);

    _trajReady = false;
    _odomReady = false;
    _started = false;
    _completed = false;

    double k1 = _nh.param<double>("k1", 100.0);
    double k2 = _nh.param<double>("k2", 100.0);
    _filter.initFilterStep(0.001, k1, k2, Vector2d::Zero(), Vector2d::Zero());
    _filterSteps = _nh.param<double>("filterSteps", 1);

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
        return;
    }

    // Retrieve current trajectory point
    _dp = _traj.p[steps];
    _dv = _traj.v[steps];
    _da = _traj.a[steps];
}


void Controller::_outerLoop(){
    /* Compute position and linear velocity error */
    geometry_msgs::Pose& pose = this->_odom.pose.pose;
    geometry_msgs::Twist& twist = this->_odom.twist.twist;

    // Position error
    Vector3d ep (pose.position.x - _dp.position.x,
            pose.position.y - _dp.position.y,
            pose.position.z - _dp.position.z);

    // Transform velocity from body frame to worldNED frame
    Vector3d linVel (twist.linear.x, twist.linear.y, twist.linear.z);
    linVel = _Rb * linVel;

    // Linear velocity error
    Vector3d epd (linVel.x() - _dv.linear.x,
            linVel.y() - _dv.linear.y,
            linVel.z() - _dv.linear.z);

    // Compute mu_d
    Matrix<double,6,1> e;
    e << ep, epd;
    _epInt += e / _rate;

    _mud = -_Kp*e - _Kpi*_epInt + Vector3d(_da.linear.x, _da.linear.y, _da.linear.z);

    // Compute outputs
    _uT = _m * sqrt(pow(_mud[0],2) + pow(_mud[1],2) + pow(_mud[2] - GRAVITY,2));

    _deta[0] = asin(_m * (_mud[1]*cos(_dp.yaw) - _mud[0]*sin(_dp.yaw)) / _uT);
    _deta[1] = atan2(_mud[0]*cos(_dp.yaw) + _mud[1]*sin(_dp.yaw), _mud[2] - GRAVITY);
    _deta[1] += M_PI * ((_deta[1] > 0) ? -1 : 1);
    _deta[2] = _dp.yaw;
}


void Controller::_innerLoop(){
    // Compute derivatives of orientation
    Vector3d deta_d, deta_dd;
    this->_filter.filterSteps(_deta.segment<2>(0), _filterSteps);
    deta_d << _filter.lastFirst(), _dv.angular.z;
    deta_dd << _filter.lastSecond(), _da.angular.z;

    // Compute orientation errors
    geometry_msgs::Pose& pose = this->_odom.pose.pose;
    geometry_msgs::Twist& twist = this->_odom.twist.twist;
    double r,p,y;
    tf::Matrix3x3(tf::Quaternion(pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w)).getRPY(r,p,y);

    Vector3d eta(r,p,y);
    Vector3d eo = eta - _deta;

    // Compute Q matrix
    Matrix3d Q, QT, Q_inv, Q_dot;
    Q << 1,     0,              -sin(eta[0]),
         0,     cos(eta[0]),    cos(eta[1])*sin(eta[0]),
         0,     -sin(eta[0]),   cos(eta[1])*cos(eta[0]);
    QT = Q.transpose();
    Q_inv = Q.inverse();
    Q_dot << 0, 0,              -cos(eta[0]),
             0, -sin(eta[0]),   -sin(eta[1])*sin(eta[0])+cos(eta[1])*cos(eta[0]),
             0, -cos(eta[0]),   -sin(eta[1])*cos(eta[0])-cos(eta[1])*sin(eta[0]);

    Vector3d omega (twist.angular.x, twist.angular.y, twist.angular.z);
    Vector3d eta_d = Q_inv * omega;
    Vector3d eod = eta_d - deta_d;

    Matrix<double,6,1> e;
    e << eo, eod;
    _eoInt += e / _rate;

    // Compute Coriolis matrix
    Matrix3d C = QT * skew(omega) * _Ib * Q  +  QT * _Ib * Q_dot;

    // Compute tau
    Vector3d tauTilde = -_Ke*e - _Kei*_eoInt + deta_dd;
    _tau = _Ib*Q*tauTilde + Q.inverse().transpose() * C * eta_d;
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
            cmd.torque.x = _tau[0];
            cmd.torque.y = _tau[1];
            cmd.torque.z = _tau[2];

            _pub.publish(cmd);
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
