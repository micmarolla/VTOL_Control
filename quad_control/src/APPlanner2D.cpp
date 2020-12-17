#include "APPlanner2D.h"
#include <cmath>
#include <iostream>
#include <vector>
#include "nav_msgs/Path.h"
#include "geometry_msgs/Accel.h"
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
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

    _sub = _nh.subscribe("/planRequest", 0, &APPlanner2D::plan, this);
    _pub = _nh.advertise<quad_control::Trajectory>("/trajectory", 0, true);
    _pathPub = _nh.advertise<nav_msgs::Path>("/plannedPath", 0, true);

}


void APPlanner2D::setMap(nav_msgs::OccupancyGrid &map){
    this->_mapAnalyzer.setMap(map);
    this->_mapAnalyzer.scan();
}


Vector6d APPlanner2D::_computeError(geometry_msgs::Pose q, geometry_msgs::Pose qd){
    Vector6d e;
    
    // Position
    e[0] = qd.position.x - q.position.x;
    e[1] = qd.position.y - q.position.y;
    e[2] = qd.position.z - q.position.z;
    
    // Orientation: from quaternion to RPY
    double r,p,y, rd,pd,yd;
    tf::Matrix3x3(tf::Quaternion(q.orientation.x, q.orientation.y,
        q.orientation.z, q.orientation.w)).getRPY(r,p,y);
    tf::Matrix3x3(tf::Quaternion(qd.orientation.x, qd.orientation.y,
        qd.orientation.z, qd.orientation.w)).getRPY(rd,pd,yd);

    e[3] = rd - r;
    e[4] = pd - p;
    e[5] = yd - y;

    return e;
}



geometry_msgs::Pose APPlanner2D::_eulerIntegration(geometry_msgs::Pose q,
        Vector6d ft){

    /* Position */
    q.position.x = q.position.x + this->_sampleTime * ft[0];
    q.position.y = q.position.y + this->_sampleTime * ft[1];
    q.position.z = q.position.z + this->_sampleTime * ft[2];

    /* Orientation */

    // Retrieve RPY angles
    double r,p,y;
    tf::Matrix3x3(tf::Quaternion(q.orientation.x, q.orientation.y,
        q.orientation.z, q.orientation.w)).getRPY(r,p,y);

    // Compute integration
    r = r + this->_sampleTime * ft[3];
    p = p + this->_sampleTime * ft[4];
    y = y + this->_sampleTime * ft[5];

    // Go back to quaternions
    tf::Quaternion quat;
    quat.setRPY(r,p,y);
    tf::quaternionTFToMsg(quat, q.orientation);

    return q;    
}


Vector6d APPlanner2D::_computeForce(geometry_msgs::Pose q, Vector6d e){
    Vector6d fa, fr;

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


Vector6d APPlanner2D::_computeRepulsiveForce(double rx, double ry){
    Vector6d fr;
    fr << 0,0,0,0,0,0;
    double fri_mod = 0;
    Eigen::Vector2d fri;

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
    if(!this->_mapAnalyzer.mapReady() || (req->map.info.width > 0 && req->map.info.height > 0))
        this->setMap(req->map);

    // Check that repulsive forces in q_goal are null
    Vector6d fr_g = _computeRepulsiveForce(req->qg.position.x, req->qg.position.y);
    if(fr_g[0] != 0 || fr_g[1] != 0){
        ROS_ERROR("Repulsive forces in goal configuration are not null. Planning is not possible");
        return;
    }

    // Build path msg
    quad_control::Trajectory trajectory;
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "world";

    geometry_msgs::PoseStamped q;
    geometry_msgs::Accel v;
    q.pose = req->qs;
    Vector6d err, ft, ftPrev;
    bool done = false, first = true;

    while (!done){
        // Compute error, force, integrate and add to path
        err = _computeError(q.pose, req->qg);
        ft = _computeForce(q.pose, err);
        q.pose = _eulerIntegration(q.pose, ft);
        
        // Accelerations
        if(first)
            first = false;
        else{
            v.linear.x  = (ft[0]-ftPrev[0])/_sampleTime;
            v.linear.z  = (ft[1]-ftPrev[1])/_sampleTime;
            v.linear.y  = (ft[2]-ftPrev[2])/_sampleTime;
            v.angular.x = (ft[3]-ftPrev[3])/_sampleTime;
            v.angular.z = (ft[4]-ftPrev[4])/_sampleTime;
            v.angular.y = (ft[5]-ftPrev[5])/_sampleTime;
            trajectory.a.push_back(v);
        }
        ftPrev = ft;

        // Velocities
        v.linear.x = ft[0]; v.linear.y = ft[1]; v.linear.z = ft[2];
        v.angular.x = ft[3]; v.angular.y = ft[4]; v.angular.z = ft[5];
        trajectory.v.push_back(v);

        // Positions
        q.header.stamp = ros::Time::now();
        q.header.frame_id = "world";
        path.poses.push_back(q);
        trajectory.p.push_back(q.pose);

        // Check if goal has been reached
        done = (Eigen::Vector3d(err[0],err[1],err[2]).norm() <= this->_p_eps)
                && (Eigen::Vector3d(err[3],err[4],err[5]).norm() <= this->_o_eps);
    }

    // Append final null acceleration
    v.linear.x = v.linear.y = v.linear.z = v.angular.x = v.angular.y = v.angular.z = 0;
    trajectory.a.push_back(v);
    
    trajectory.sampleTime = this->_sampleTime;
    
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
