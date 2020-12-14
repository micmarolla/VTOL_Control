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
    _eta        = _nh.param<double>("eta", 2.0);
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
    tf::Matrix3x3(tf::Quaternion(q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w)).getRPY(r,p,y);
    tf::Matrix3x3(tf::Quaternion(qd.orientation.x, qd.orientation.y, qd.orientation.z, qd.orientation.w)).getRPY(r,p,y);

    e[3] = rd - r;
    e[4] = pd - p;
    e[5] = yd - y;

    return e;
}



geometry_msgs::Pose APPlanner2D_Server::_eulerIntegration(geometry_msgs::Pose q, Vector6d ft){
    /* Position */
    q.position.x = q.position.x + this->_sampleTime * ft[0];
    q.position.y = q.position.y + this->_sampleTime * ft[1];
    q.position.z = q.position.z + this->_sampleTime * ft[2];

    /* Orientation */

    // Retrieve RPY angles
    double r,p,y;
    tf::Matrix3x3(tf::Quaternion(q.orientation.x, q.orientation.y, q.orientation.z, q.orientation.w)).getRPY(r,p,y);

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
    /*int subW, subH, x, y;
    shared_ptr<int8_t[]> submap = _getNeighbourhood(this->_map,
        Eigen::Vector3d(q.position.x, q.position.y, q.position.z),
        subW, subH, x, y);*/
    fr = _computeRepulsiveForce(/*submap, subW, subH,*/ q.position.x, q.position.y);
    //submap.reset();

    return fa + fr;
}


/*std::shared_ptr<int8_t[]> APPlanner2D_Server::_getNeighbourhood(Eigen::Vector3d pos, int &w, int &h, int &x, int &y){
    // Retrieve robot position in map
    int rx = ceil((pos[0] - _map.info.origin.position.x) / _map.info.resolution);
    int ry = ceil((pos[1] - _map.info.origin.position.y) / _map.info.resolution);

    // Neighbourhood span (in cells)
    int cellSpan = ceil(this->_eta / grid.info.resolution);
    //cout << "cellSpan: " << cellSpan << endl;

    // Get neighbourhood corners
    int x1 = rx-cellSpan, x2 = rx+cellSpan;
    int y1 = ry-cellSpan, y2 = ry+cellSpan;

    // Fix pos if near margins
    if (x1 < 0)                 x1 = 0;
    if (x2 > grid.info.width)   x2 = grid.info.width;
    if (y1 < 0)                 y1 = 0;
    if (y2 > grid.info.height)  y2 = grid.info.height;

    // Retrieve data
    int size = (x2-x1+1) * (y2-y1+1);
    std::shared_ptr<int8_t[]> submap(new int8_t[size]);
    for (int i=0; i < size; ++i){
        int _x = x1 + i % (x2-x1);
        int _y = y1 + i / (y2-y1);
        submap[i] = grid.data[_x + _y*grid.info.width];
    }

    // Robot position [cells] in submap
    x = rx - x1;
    y = ry - y1;

    return submap;
}*/


Vector6d APPlanner2D_Server::_computeRepulsiveForce(/*shared_ptr<int8_t[]> submap, int w, int h,*/ int rx, int ry){
    Vector6d fr;
    fr << 0,0,0,0,0,0;
    double fri_mod = 0;
    Eigen::Vector2d fri;

    // For each chunk, considering the minimum-distance point from the robot...
    vector<Chunk*> obstacles = this->_mapAnalyzer.getObjAtMinDist(rx, ry);
    for (auto obj : obstacles){
        if (obj->dist2 > pow(this->_eta,2))
            continue;

        // ... compute repulsive force
        fri_mod = (_kr / obj->dist2) * pow(1/sqrt(obj->dist2) - 1/_eta, _gamma - 1);
        fri = fri_mod * Eigen::Vector2d(rx - obj->x, ry - obj->y).normalized();
    
        fr[0] += fri[0];
        fr[1] += fri[1];
    }

    return fr;
}


bool APPlanner2D_Server::plan(quad_control::APPlanner2D::Request &req, quad_control::APPlanner2D::Response &res){
    ROS_INFO("APPlanner_2D: path planning requested.");

    // Update map
    if(!this->_mapReady) // If no map is present
        this->setMap(req.map);

    // Build path msg
    res.path.header.stamp = ros::Time::now();
    res.path.header.frame_id = "worldNED";

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
        q.header.frame_id = "worldNED";

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
