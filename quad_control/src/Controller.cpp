#include "Controller.h"
#include <cmath>
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf_conversions/tf_eigen.h>
#include "geometry_msgs/Wrench.h"
#include "Utils.h"

#define _P_EPS  0.005
#define _O_EPS  0.01


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

    _trajStep = 0;
    _remainingSteps = 0;
    _doneSteps = 0;
    _trajReady = false;

    _odomReady = false;
    _started = false;
    _completed = false;

    _uT = 0;
    _tau << 0,0,0;

    double k1 = _nh.param<double>("k1", 100.0);
    double k2 = _nh.param<double>("k2", 100.0);
    _filter.initFilterStep(0.001, k1, k2, Vector2d::Zero(), Vector2d::Zero());
    _filterSteps = _nh.param<double>("filterSteps", 1);

    _trajSub = _nh.subscribe("/trajectory", 0, &Controller::trajectoryReceived, this);
    _odomSub = _nh.subscribe("/hummingbird/ground_truth/odometryNED", 0, &Controller::odomReceived, this);
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
    if(_completed)
        return;
        
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
        _completed = true;
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
    //ROS_INFO("Outer loop");

    // Position error
    Vector3d ep = _p - Vector3d(_dp.position.x, _dp.position.y, _dp.position.z);
    //ROS_INFO_STREAM("    Pos error: " << ep[0] << ", " << ep[1] << ", " << ep[2]);

    // Linear velocity error
    Vector3d epd = _p_d - Vector3d(_dv.linear.x, _dv.linear.y, _dv.linear.z);
    //ROS_INFO_STREAM("    Lin vel error: " << epd[0] << ", " << epd[1] << ", " << epd[2]);

    // Compute mu_d
    _e_p << ep, epd;
    _epInt += _e_p / _rate;

    _computeMu();
    //ROS_INFO_STREAM("    Mu: " << _mud[0] << ", " << _mud[1] << ", " << _mud[2]);

    // Compute outputs
    _uT = _m * sqrt(pow(_mud[0],2) + pow(_mud[1],2) + pow(_mud[2] - GRAVITY,2));

    _deta[0] = asin(_m * (_mud[1]*cos(_dp.yaw) - _mud[0]*sin(_dp.yaw)) / _uT);
    _deta[1] = atan2(_mud[0]*cos(_dp.yaw) + _mud[1]*sin(_dp.yaw), _mud[2] - GRAVITY);
    _deta[1] += M_PI * ((_deta[1] > 0) ? -1 : 1);
    _deta[2] = _dp.yaw;
    //ROS_INFO_STREAM("    des_eta: " << _deta[0] << ", " << _deta[1] << ", " << _deta[2]);
}


void Controller::_innerLoop(){
    //ROS_INFO("Inner loop");

    // Compute derivatives of orientation
    this->_filter.filterSteps(_deta.head<2>(), _filterSteps);
    _deta_d << _filter.lastFirst(), _dv.angular.z;
    _deta_dd << _filter.lastSecond(), _da.angular.z;
    //ROS_INFO_STREAM("    Filter vel: " << _deta_d[0] << ", " << _deta_d[1] << ", " << _deta_d[2]);
    //ROS_INFO_STREAM("    Filter acc: " << _deta_dd[0] << ", " << _deta_dd[1] << ", " << _deta_dd[2]);

    // Compute orientation errors
    Vector3d eo = _eta - _deta;
    Vector3d eod = _eta_d - _deta_d;
    //ROS_INFO_STREAM("    Orient err: " << eo[0] << ", " << eo[1] << ", " << eo[2]);
    //ROS_INFO_STREAM("    Orient dot err: " << eod[0] << ", " << eod[1] << ", " << eod[2]);

    _e_eta << eo, eod;
    _eoInt += _e_eta / _rate;

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
        if(_trajReady && _odomReady){
            if(!_started){
                _startTime = ros::Time::now();
                _started = true;
                ROS_INFO("Starting trajectory");
            }

            _getCurrentTrajPoint();
            _coreLoop();

            //ROS_INFO_STREAM("uT = " << _uT);
            //ROS_INFO_STREAM("tau: " << _tau[0] << ", " << _tau[1] << ", " << _tau[2]);
            //ROS_INFO("=========================================");

            // Build command msg and publish it
            cmd.force.z = _uT;
            cmd.torque.x = _tau[0];
            cmd.torque.y = _tau[1];
            cmd.torque.z = _tau[2];

            _pub.publish(cmd);

            pnt.header.stamp = ros::Time::now();
            pnt.point = _dp.position;
            _pointPub.publish(pnt);
        }

        r.sleep();
        ros::spinOnce();
    }
}
