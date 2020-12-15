#include "APPlanner2D_Server.h"
#include <cmath>
#include <iostream>
#include <vector>
#include "nav_msgs/Path.h"
#include <tf/tf.h>
#include <tf/LinearMath/Matrix3x3.h>
#include "MapAnalyzer.h"

using namespace std;

APPlanner2D_Server::APPlanner2D_Server() : _nh("~"){
    // Retrieve params
    _ka         = _nh.param<double>("ka", 1.0);
    _kr         = _nh.param<double>("kr", 1.0);
    _eta        = _nh.param<double>("eta", 1.0);
    _gamma      = _nh.param<double>("gamma", 2.0);
    _p_eps      = _nh.param<double>("p_eps", 0.001);
    _o_eps      = _nh.param<double>("o_eps", 0.001);
    _sampleTime = _nh.param<double>("sampleTime", 0.01);

    _mapReady = false;

    _server = _nh.advertiseService("/planning_srv", &APPlanner2D_Server::plan, this);
}


void APPlanner2D_Server::setMap(nav_msgs::OccupancyGrid &map){
    this->_map = map;
    this->_mapAnalyzer.analyze(map);
    this->_mapReady = true;
}


Vector6d APPlanner2D_Server::_computeError(geometry_msgs::Pose q, geometry_msgs::Pose qd){
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



geometry_msgs::Pose APPlanner2D_Server::_eulerIntegration(geometry_msgs::Pose q,
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


Vector6d APPlanner2D_Server::_computeForce(geometry_msgs::Pose q, Vector6d e){
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


Vector6d APPlanner2D_Server::_computeRepulsiveForce(double rx, double ry){
    Vector6d fr;
    fr << 0,0,0,0,0,0;
    double fri_mod = 0;
    Eigen::Vector2d fri;

    // Retrieve robot coordinates in the map cell reference
    int rxCell = (rx - this->_map.info.origin.position.x) / this->_map.info.resolution;
    int ryCell = (ry - this->_map.info.origin.position.y) / this->_map.info.resolution;

    // For each chunk, considering the minimum-distance point from the robot...
    vector<Chunk*> obstacles = this->_mapAnalyzer.getObjAtMinDist(rxCell, ryCell);
    for (auto obj : obstacles){

        obj->dist2 *= pow(this->_map.info.resolution, 2); // convert to [meters^2]
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


bool APPlanner2D_Server::plan(quad_control::APPlanner2D::Request &req,
        quad_control::APPlanner2D::Response &res){

    ROS_INFO("APPlanner_2D: path planning requested.");

     // If no map is present, or the current map if updated
    if(!this->_mapReady || (req.map.info.width > 0 && req.map.info.height > 0))
        this->setMap(req.map);

    // Check that repulsive forces in q_goal are null
    Vector6d fr_g = _computeRepulsiveForce(req.qg.position.x, req.qg.position.y);
    if(fr_g[0] != 0 || fr_g[1] != 0){
        ROS_ERROR("Repulsive forces in goal configuration are not null. Planning is not possible");
        return false;
    }

    // Build path msg
    res.path.header.stamp = ros::Time::now();
    res.path.header.frame_id = "world";

    geometry_msgs::PoseStamped q;
    q.pose = req.qs;
    Vector6d err, ft;
    bool done = false;

    while (!done){
        // Compute error, force, integrate and add to path
        err = _computeError(q.pose, req.qg);
        ft = _computeForce(q.pose, err);
        q.pose = _eulerIntegration(q.pose, ft);
        
        q.header.stamp = ros::Time::now();
        q.header.frame_id = "world";

        res.path.poses.push_back(q);

        // Check if goal has been reached
        done = (Eigen::Vector3d(err[0],err[1],err[2]).norm() <= this->_p_eps)
                && (Eigen::Vector3d(err[3],err[4],err[5]).norm() <= this->_o_eps);
    }
    
    ROS_INFO("APPlanner_2D: path planning successfully completed.");

    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner");
    APPlanner2D_Server server;
    ros::spin();
    return 0;
}
