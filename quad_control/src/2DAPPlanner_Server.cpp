#include "2DAPPlanner_Server.h"
#include "nav_msgs/Path.h"
#include <tf/tf.h>

2DAPPlanner_Server::2DAPPlanner_Server() : _nh("~"){
    // Retrieve params
    _ka         = _nh.param<double>("ka", 1);
    _kr         = _nh.param<double>("kr", 1);
    _eta        = _nh.param<double>("eta", 1.5);
    _gamma      = _nh.param<double>("gamma", 2);
    _p_eps      = _nh.param<double>("p_eps", 0.01);
    _o_eps      = _nh.param<double>("o_eps", 0.01);
    _sampleTime = _nh.param<double>("sampleTime", 0.01);

    _server = _nh.advertiseService("planning_srv", 2DAPPlanner_Server::plan);
}


Vector6d 2DAPPlanner_Server::_computeError(geometry_msgs::Pose q, geometry_msgs::Pose qd){
    Vector6d e;
    
    // Position
    e << q.position.x - qd.position.x, q.position.y - qd.position.y, q.position.z - qd.position.z;
    
    // Orientation: from quaternion to RPY
    double r,p,y, rd,pd,yd;
    tf::Quaternion(q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w).getRPY(r,p,y);
    tf::Quaternion(qd.orientation.x, qd.orientation.y, qd.orientation.z, qd.orientation.w).getRPY(r,p,y);

    e << r - rd, p - pd, y - yd;

    return e;
}



geometry_msgs::Pose 2DAPPlanner_Server::_eulerIntegration(geometry_msgs::Pose q, Vector6d ft){
    /* Position */
    q.position.x = q.position.x + this->_sampleTime * ft[0];
    q.position.y = q.position.y + this->_sampleTime * ft[1];
    q.position.z = q.position.z + this->_sampleTime * ft[2];

    /* Orientation */

    // Retrieve RPY angles
    double r,p,y;
    tf::Quaternion(q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w).getRPY(r,p,y);

    // Compute integration
    r = r + this->_sampleTime * ft[3];
    p = p + this->_sampleTime * ft[4];
    y = y + this->_sampleTime * ft[5];

    // Go back to quaternions
    tf::Quaternion quat;
    quat.setRPY(r,p,y);
    q.orientation.x = quat.x;
    q.orientation.y = quat.y;
    q.orientation.z = quat.z;
    q.orientation.w = quat.w;

    return q;    
}


Vector6d 2DAPPlanner_Server::_computeForce(Vector6d e){
    Vector6d ft;
    // ...
    return ft;
}


bool 2DAPPlanner_Server::plan(quad_control::2DAPPlanner::Request &req, quad_control::2DAPPlanner::Response &res){
    // Build path msg
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "worldNED";

    geometry_msgs::Pose q = req.qs;
    Vector6d err, ft;
    bool done = false;

    while (!done){
        // Compute error, force, integrate and add to path
        err = _computeError(q, re.qg);
        ft = _computeForce(err);
        q = _eulerIntegration(q, ft);
        path.poses.push_back(q);

        // Check if goal has been reached
        done = (Eigen::Vector3d(err[0],err[1],err[2]).norm() <= this->_p_eps)
                && (Eigen::Vector3d(err[3],err[4],err[5]).norm() <= this->_o_eps);
    }

    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner");
    2DAPPlanner_Server server;
    ROS_INFO_NAMED("2DAPPlanner_Server", "Ready.");
    ros::spin();
    return 0;
}
