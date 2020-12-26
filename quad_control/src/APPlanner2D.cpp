#include "APPlanner2D.h"
#include <cmath>
#include <iostream>
#include <vector>
#include "nav_msgs/Path.h"
#include "geometry_msgs/Accel.h"
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include "MapAnalyzer.h"
#include "quad_control/PlanRequest.h"
#include "quad_control/Trajectory.h"

using namespace std;

APPlanner2D::APPlanner2D() : _nh("~"){
    // Retrieve params
    _ka         = _nh.param<double>("ka", 1.0);
    _kr         = _nh.param<double>("kr", 1.0);
    _eta        = _nh.param<double>("eta", 1.0);
    _gamma      = _nh.param<double>("gamma", 2.0);
    _p_eps      = _nh.param<double>("p_eps", 0.001);
    _o_eps      = _nh.param<double>("o_eps", 0.001);
    _sampleTime = _nh.param<double>("sampleTime", 0.01);
    _debugPath  = _nh.param<bool>("debugPath", true);

    _sub = _nh.subscribe("/planRequest", 0, &APPlanner2D::plan, this);
    _pub = _nh.advertise<quad_control::Trajectory>("/trajectory", 0, true);

    if(_debugPath)
        _pathPub = _nh.advertise<nav_msgs::Path>("/plannedPath", 0, true);
}


void APPlanner2D::setMap(nav_msgs::OccupancyGrid &map){
    this->_mapInfo = map.info;
    this->_mapAnalyzer.setMap(map);
    this->_mapAnalyzer.scan();
}


Eigen::Vector4d APPlanner2D::_computeError(quad_control::UAVPose q, quad_control::UAVPose qd){
    Eigen::Vector4d e;
    e << qd.position.x - q.position.x, qd.position.y - q.position.y,
            qd.position.z - q.position.z, qd.yaw - q.yaw;
    return e;
}



quad_control::UAVPose APPlanner2D::_eulerIntegration(quad_control::UAVPose q,
        Eigen::Vector4d ft){

    q.position.x = q.position.x + _sampleTime * ft[0];
    q.position.y = q.position.y + _sampleTime * ft[1];
    q.position.z = q.position.z + _sampleTime * ft[2];
    q.yaw = q.yaw + _sampleTime * ft[3];

    return q;
}


Eigen::Vector4d APPlanner2D::_computeForce(quad_control::UAVPose q, Eigen::Vector4d e){
    Eigen::Vector4d fa, fr;

    /* Attractive potentials */
    if (e.norm() <= 1)
        // Paraboloid
        fa = this->_ka * e;
    else
        // Conical
        fa = this->_ka * e / e.norm();

    /* Repulsive potentials */
    fr = _computeRepulsiveForce(q.position.x, q.position.y);

    return fa + fr;
}


Eigen::Vector4d APPlanner2D::_computeRepulsiveForce(double rx, double ry){
    Eigen::Vector4d fr;
    fr << 0,0,0,0;
    Eigen::Vector2d fri;
    double fri_mod = 0;

    // Retrieve robot coordinates in the map cell reference
    int rxCell = (rx - this->_mapInfo.origin.position.x) / this->_mapInfo.resolution;
    int ryCell = (ry - this->_mapInfo.origin.position.y) / this->_mapInfo.resolution;

    // For each chunk, considering the minimum-distance point from the robot...
    vector<Chunk*> obstacles = this->_mapAnalyzer.getObjAtMinDist(rxCell, ryCell);
    for (auto obj : obstacles){
        obj->dist2 *= pow(this->_mapInfo.resolution, 2); // convert to [meters^2]
        if (obj->dist2 > pow(this->_eta,2))
            continue;

        // ... compute repulsive force
        fri_mod = (_kr / obj->dist2) * pow(1/sqrt(obj->dist2) - 1/_eta, _gamma - 1);
        fri = fri_mod * Eigen::Vector2d(rxCell - obj->x, ryCell - obj->y).normalized();

        fr[0] += fri[0];
        fr[1] += fri[1];
    }

    return fr;
}


void APPlanner2D::plan(quad_control::PlanRequestPtr req){
    ROS_INFO("APPlanner_2D: path planning requested.");

     // If no map is present, or the current map has been updated
    if(!this->_mapAnalyzer.ready() || (req->map.info.width > 0 && req->map.info.height > 0))
        this->setMap(req->map);

    // Check that repulsive forces in q_goal are null
    Eigen::Vector4d fr_g = _computeRepulsiveForce(req->qg.position.x, req->qg.position.y);
    if(fr_g[0] != 0 || fr_g[1] != 0){
        ROS_ERROR("Repulsive forces in goal configuration are not null. Planning is not possible");
        return;
    }

    // Build path msg
    quad_control::Trajectory trajectory;
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "worldNED";

    quad_control::UAVPose q = req->qs;
    geometry_msgs::Accel v;
    v.angular.x = v.angular.y = 0;

    Eigen::Vector4d err, ft, ftPrev;
    bool done = false, first = true;

    while (!done){
        // Compute error, force, integrate and add to path
        err = _computeError(q, req->qg);
        ft = _computeForce(q, err);
        q = _eulerIntegration(q, ft);

        // Accelerations
        if(first)
            first = false;
        else{
            v.linear.x  = (ft[0]-ftPrev[0])/_sampleTime;
            v.linear.y  = (ft[1]-ftPrev[1])/_sampleTime;
            v.linear.z  = (ft[2]-ftPrev[2])/_sampleTime;
            v.angular.z = (ft[3]-ftPrev[3])/_sampleTime;
            trajectory.a.push_back(v);
        }
        ftPrev = ft;

        // Velocities
        v.linear.x = ft[0]; v.linear.y = ft[1]; v.linear.z = ft[2];
        v.angular.z = ft[3];
        trajectory.v.push_back(v);

        // Positions
        trajectory.p.push_back(q);

        // Debug path
        if(_debugPath){
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "worldNED";
            pose.pose.position = q.position;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(q.yaw);
            path.poses.push_back(pose);
        }

        // Check if goal has been reached
        done = (Eigen::Vector3d(err[0],err[1],err[2]).norm() <= this->_p_eps
                && err[3] <= this->_o_eps);
    }

    // Append final entry with null velocity and acceleration, and same position
    v.linear.x = v.linear.y = v.linear.z = 0;
    v.angular.x = v.angular.y = v.angular.z = 0;
    trajectory.p.push_back(trajectory.p.back());
    trajectory.v.push_back(v);
    trajectory.a.push_back(v);  // Twice, for having the same number of points
    trajectory.a.push_back(v);

    trajectory.sampleTime = this->_sampleTime;

    if(_debugPath)
        this->_pathPub.publish(path);
    this->_pub.publish(trajectory);

    ROS_INFO("APPlanner_2D: path planning successfully completed.");
}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner");
    APPlanner2D planner;
    ros::spin();
    return 0;
}
