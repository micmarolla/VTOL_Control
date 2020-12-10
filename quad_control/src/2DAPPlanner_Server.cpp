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


geometry_msgs::Pose 2DAPPlanner_Server::_eulerIntegration(geometry_msgs::Pose q){
    /* Position */
    q.position.x = q.position.x + this->_sampleTime * ft;
    q.position.y = q.position.y + this->_sampleTime * ft;
    q.position.z = q.position.z + this->_sampleTime * ft;

    /* Orientation */

    // Retrieve RPY angles
    double r,p,y;
    tf::Quaternion(q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w).getRPY(r,p,y);

    // Compute integration
    r = r + this->_sampleTime * ft;
    p = p + this->_sampleTime * ft;
    y = y + this->_sampleTime * ft;

    // Go back to quaternions
    tf::Quaternion quat;
    quat.setRPY(r,p,y);
    q.orientation.x = quat.x;
    q.orientation.y = quat.y;
    q.orientation.z = quat.z;
    q.orientation.w = quat.w;
}


double 2DAPPlanner_Server::computeForce(geometry_msgs::Pose q, geometry_msgs::Pose qg){
    // ...
    return 0;
}


bool 2DAPPlanner_Server::plan(quad_control::2DAPPlanner::Request &req, quad_control::2DAPPlanner::Response &res){
    // Build path msg
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "worldNED";

    geometry_msgs::Pose q = req.qs;
    bool done = false;

    while (!done){
        // Compute force, integrate and add to path
        double ft = computeForce(q, req.qg);
        q = this->_eulerIntegration(q);
        path.poses.push_back(q);

        // Check if goal has been reached
        double pErrorNorm = sqrt(pow(q.position.x - qg.position.x, 2) +
            pow(q.position.y - qg.position.y, 2) +
            pow(q.position.z - qg.position.z, 2));

        double r,p,y;
        double rd,pd,yd;
        tf::Quaternion(q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w).getRPY(r,p,y);
        tf::Quaternion(qg.orientation.x, qg.orientation.y, qg.orientation.z, qg.orientation.w).getRPY(rd,pd,yd);
        double oErrorNorm = sqrt(pow(r-rd, 2) + pow(p-pd, 2) + pow(y-yd, 2));

        done = (pErrorNorm <= this->_p_eps) && (oErrorNorm <= this->_o_eps);
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
