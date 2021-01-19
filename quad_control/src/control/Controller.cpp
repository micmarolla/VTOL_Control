#include "Controller.h"

#include <cmath>

#include <geometry_msgs/Wrench.h>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>

#include "common.h"
#include "Utils.h"

using namespace Eigen;

Controller::Controller() : _nh("~"){
    _rate = _nh.param<double>("rate", 1000.0);

    // Mass and inertia params. Default values are for AscTec Hummingbird UAV
    _m = _nh.param<double>("m", 0.68);
    _Ib = Matrix3d::Identity();
    _Ib(0,0) = _nh.param<double>("Ibx", 0.007);
    _Ib(1,1) = _nh.param<double>("Iby", 0.007);
    _Ib(2,2) = _nh.param<double>("Ibz", 0.012);

    // Proportional, derivative, integral gains
    double kp = _nh.param<double>("kp", 10.0);
    double kpv = _nh.param<double>("kpd", 10.0);
    double ke = _nh.param<double>("ko", 1.0);
    double kev = _nh.param<double>("kod", 1.0);
    _Kp << kp*Matrix3d::Identity(), kpv*Matrix3d::Identity();
    _Ke << ke*Matrix3d::Identity(), kev*Matrix3d::Identity();

    double kpi = _nh.param<double>("kpi", 0.0);
    double kei = _nh.param<double>("kei", 0.0);
    _Kpi << kpi*Matrix3d::Identity();
    _Kei << kei*Matrix3d::Identity();

    // Filtering params
    double k1 = _nh.param<double>("k1", 100.0);
    double k2 = _nh.param<double>("k2", 100.0);
    double filterRate = _nh.param<double>("filterRate", 100.0);
    _filter.initFilterStep(1/filterRate, k1, k2, Vector2d::Zero(), Vector2d::Zero());
    _filterSteps = _nh.param<int>("filterSteps", 1);

    _epInt = Vector3d::Zero();
    _eoInt = Vector3d::Zero();

    _trajStep = 0;
    _remainingSteps = 0;

    _trajReady = false;
    _odomReady = false;
    _waiting   = true;
    _tracking  = false;
    _landing   = false;

    _uT  = 0;
    _tau = Vector3d::Zero();

    _trajSub = _nh.subscribe("/trajectory", 0, &Controller::trajectoryReceived, this);
    _odomSub = _nh.subscribe("/hummingbird/odometryNED", 0, &Controller::odomReceived, this);
    _pub = _nh.advertise<geometry_msgs::Wrench>("/hummingbird/command/wrenchNED", 0);
    _pointPub = _nh.advertise<geometry_msgs::PointStamped>("/trajectoryPoint", 0);
}


void Controller::trajectoryReceived(quad_control::TrajectoryPtr traj){
    _traj = *traj;
    _trajReady = true;
}

void Controller::odomReceived(nav_msgs::OdometryPtr odom){
    _odom = *odom;

    geometry_msgs::Pose& pose = _odom.pose.pose;
    geometry_msgs::Twist& twist = _odom.twist.twist;

    /* Position and linear velocity */
    _p << pose.position.x, pose.position.y, pose.position.z;
    // Transform velocity from body frame to worldNED frame
    _p_d << twist.linear.x, twist.linear.y, twist.linear.z;
    _p_d = _Rb * _p_d;

    /* Orientation and angular velocity */
    // Get current rotation matrix of body frame w.r.t. worldNED frame
    tf::Matrix3x3 tfRb (tf::Quaternion(pose.orientation.x, pose.orientation.y,
        pose.orientation.z, pose.orientation.w));
    tf::matrixTFToEigen(tfRb, _Rb);

    // Retrieve rpy angles
    double r,p,y;
    tfRb.getRPY(r,p,y);
    _eta << r,p,y;

    // Compute Q matrix
    _computeQ(_eta);

    // Compute angular velocity and eta_dot
    _omega << twist.angular.x, twist.angular.y, twist.angular.z;
    _eta_d = _Q_inv * _omega;

    // Compute Coriolis matrix
    _computeC(_omega);

    _odomReady = true;
}


void Controller::_getCurrentTrajPoint(){
    if(!_tracking)
        return;

    // Compute steps to simulate until the next trajectory point
    if(_remainingSteps <= 0)
        _remainingSteps = ceil(_traj.t[_trajStep++] * _rate);

    // Retrieve current trajectory point
    _dp = _traj.p[_trajStep];
    _dv = _traj.v[_trajStep];
    _da = _traj.a[_trajStep];
    --_remainingSteps;

    if(_traj.p.size() <= _trajStep || _traj.v.size() <= _trajStep
            || _traj.a.size() <= _trajStep || _traj.t.size() <= _trajStep){
        ROS_INFO("Trajectory completed!");
        _tracking = false;
        _landing = true;
        return;
    }
}


void Controller::_computeQ(Vector3d eta){
    _Q << 1,     0,              -sin(eta[0]),
          0,     cos(eta[0]),    cos(eta[1])*sin(eta[0]),
          0,     -sin(eta[0]),   cos(eta[1])*cos(eta[0]);
    _QT = _Q.transpose();
    _Q_inv = _Q.inverse();
    _Q_dot << 0, 0,              -cos(eta[0]),
              0, -sin(eta[0]),   -sin(eta[1])*sin(eta[0])+cos(eta[1])*cos(eta[0]),
              0, -cos(eta[0]),   -sin(eta[1])*cos(eta[0])-cos(eta[1])*sin(eta[0]);
}

void Controller::_computeC(Vector3d omega){
    _C = _QT * skew(omega) * _Ib * _Q  +  _QT * _Ib * _Q_dot;
}


void Controller::_computeMu(){
    _mud = -_Kp * _e_p  -  _Kpi * _epInt
            + Vector3d(_da.linear.x, _da.linear.y, _da.linear.z);
}

void Controller::_computeTau(){
    Vector3d tauTilde = -_Ke*_e_eta - _Kei*_eoInt + _deta_dd;
    _tau = _Ib * _Q * tauTilde  +  _Q_inv.transpose() * _C * _eta_d;
}


void Controller::_outerLoop(){
    // Position error
    Vector3d ep = _p - Vector3d(_dp.position.x, _dp.position.y, _dp.position.z);

    // Linear velocity error
    Vector3d epd = _p_d - Vector3d(_dv.linear.x, _dv.linear.y, _dv.linear.z);

    // Compute mu_d
    _e_p << ep, epd;
    _epInt += ep / _rate;

    _computeMu();

    // Compute outputs
    _uT = _m * sqrt(pow(_mud[0],2) + pow(_mud[1],2) + pow(_mud[2] - GRAVITY,2));

    _deta[0] = asin(_m * (_mud[1]*cos(_dp.yaw) - _mud[0]*sin(_dp.yaw)) / _uT);
    _deta[1] = atan2(_mud[0]*cos(_dp.yaw) + _mud[1]*sin(_dp.yaw), _mud[2] - GRAVITY);
    _deta[1] += M_PI * ((_deta[1] > 0) ? -1 : 1);
    _deta[2] = _dp.yaw;
}


void Controller::_innerLoop(){
    // Compute derivatives of orientation
    this->_filter.filterSteps(_deta.head<2>(), _filterSteps);
    _deta_d << _filter.lastFirst(), _dv.angular.z;
    _deta_dd << _filter.lastSecond(), _da.angular.z;

    // Compute orientation errors
    Vector3d eo = _eta - _deta;
    Vector3d eod = _eta_d - _deta_d;

    _e_eta << eo, eod;
    _eoInt += eo / _rate;

    _computeTau();
}


void Controller::_coreLoop(){
    _outerLoop();       // compute uT
    _innerLoop();       // compute tau
}


void Controller::run(){
    ROS_INFO("VTOL UAV Controller started.");

    ros::Rate r(_rate);
    geometry_msgs::Wrench cmd;
    geometry_msgs::PointStamped pnt;
    pnt.header.frame_id = "worldNED";

    // These forces are not considered by the quadrotor.
    cmd.force.x = cmd.force.y = 0;

    while(ros::ok()){
        if(_waiting){
            if(_trajReady && _odomReady){
                _waiting = false;
                _tracking = true;
                ROS_INFO("Starting trajectory");
            }
        }

        else if(_tracking){
            _getCurrentTrajPoint();
            _coreLoop();
        }

        else if(_landing){
            _dv.linear.x = _dv.linear.y = _dv.linear.z =
                _dv.angular.x = _dv.angular.y = _dv.angular.z = 0;
            _da.linear.x = _da.linear.y =
                _da.angular.x = _da.angular.y = _da.angular.z = 0;

            if(_da.linear.z < 0){
                _da.linear.z += 1e-4;
                _outerLoop();
                _deta[0] = _deta[1] = 0;
                _innerLoop();
            }else{
                _uT = 0;
                _tau = Vector3d::Zero();
                _landing = false;
                _waiting = true;
                _trajReady = false;
                _odomReady = false;
                ROS_INFO("Landed.");
            }
        }

        // Build command msg and publish it
        cmd.force.z = _uT;
        cmd.torque.x = _tau[0];
        cmd.torque.y = _tau[1];
        cmd.torque.z = _tau[2];

        _pub.publish(cmd);

        pnt.header.stamp = ros::Time::now();
        pnt.point = _dp.position;
        _pointPub.publish(pnt);


        r.sleep();
        ros::spinOnce();
    }
}
