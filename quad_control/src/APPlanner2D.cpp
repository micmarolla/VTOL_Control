#include "APPlanner2D.h"
#include <cmath>
#include <vector>
#include "geometry_msgs/Accel.h"
#include <tf/transform_datatypes.h>

#define DEFAULT_STEADY_TIME     1.0     // seconds

using namespace quad_control;
using namespace Eigen;

APPlanner2D::APPlanner2D() : _nh("~"){
    // Retrieve params
    _rate       = _nh.param<double>("rate", 1.0);
    _ka         = _nh.param<double>("ka", 1.0);
    _kr         = _nh.param<double>("kr", 1.0);
    _eta        = _nh.param<double>("eta", 1.0);
    _gamma      = _nh.param<double>("gamma", 2.0);
    _p_eps      = _nh.param<double>("p_eps", 0.001);
    _o_eps      = _nh.param<double>("o_eps", 0.001);
    _sampleTime = _nh.param<double>("sampleTime", 0.01);
    _debugPath  = _nh.param<bool>("debugPath", true);

    _done = false;

    _sub = _nh.subscribe("/planRequest", 0, &APPlanner2D::plan, this);
    _pub = _nh.advertise<Trajectory>("/trajectory", 0, true);

    if(_debugPath)
        _pathPub = _nh.advertise<nav_msgs::Path>("/plannedPath", 0, false);
}


void APPlanner2D::setMap(nav_msgs::OccupancyGrid &map){
    this->_mapInfo = map.info;
    this->_mapAnalyzer.setMap(map);
    this->_mapAnalyzer.scan();
}


Vector4d APPlanner2D::_computeError(UAVPose q, UAVPose qd){
    return Vector4d (qd.position.x - q.position.x, qd.position.y - q.position.y,
            qd.position.z - q.position.z, qd.yaw - q.yaw);
}


UAVPose APPlanner2D::_eulerIntegration(UAVPose q, Vector4d ft){
    q.position.x = q.position.x + _sampleTime * ft[0];
    q.position.y = q.position.y + _sampleTime * ft[1];
    q.position.z = q.position.z + _sampleTime * ft[2];
    q.yaw = q.yaw + _sampleTime * ft[3];

    return q;
}


Vector4d APPlanner2D::_computeForce(UAVPose q, Vector4d e){
    Vector4d fa, fr;

    // Attractive potentials
    if (e.norm() <= 1)
        fa = this->_ka * e;                 // Paraboloid
    else
        fa = this->_ka * e / e.norm();      // Conical

    // Repulsive potentials
    fr = _computeRepulsiveForce(q.position.x, q.position.y);

    return fa + fr;
}


Vector4d APPlanner2D::_computeRepulsiveForce(double rx, double ry){
    Vector4d fr (0,0,0,0);
    Vector2d fri;
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
        fri = fri_mod * Vector2d(rxCell - obj->x, ryCell - obj->y).normalized();

        fr[0] += fri[0];
        fr[1] += fri[1];
    }

    return fr;
}


void APPlanner2D::_planSegment(UAVPose qs, UAVPose qg, double steadyTime,
        Trajectory& trajectory, nav_msgs::Path& path){

    UAVPose q = qs;
    geometry_msgs::Accel v;
    v.angular.x = v.angular.y = 0;

    Vector4d err, ft, ftPrev;
    bool done = false, first = true;

    while (!done){
        // Compute error, force, integrate and add to path
        err = _computeError(q, qg);
        ft = _computeForce(q, err);
        q = _eulerIntegration(q, ft);

        // Accelerations: simple numerical derivation
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
        done = (Vector3d(err[0],err[1],err[2]).norm() <= this->_p_eps
                && err[3] <= this->_o_eps);
    }

    // Append final entries with null velocity and acceleration, and same position
    for(int i=0; i < ceil(steadyTime / _sampleTime); ++i){
        v.linear.x = v.linear.y = v.linear.z = 0;
        v.angular.x = v.angular.y = v.angular.z = 0;
        trajectory.p.push_back(trajectory.p.back());
        trajectory.v.push_back(v);
        trajectory.a.push_back(v);  // Twice, for having the same number of points
        trajectory.a.push_back(v);
    }
}


void APPlanner2D::plan(PlanRequestPtr req){
    ROS_INFO("APPlanner_2D: path planning requested.");

    // Check number of points in the trajectory
    if(req->q.size() <= 1){
        ROS_ERROR("Number of points in PlanRequest must be at least 2.");
        return;
    }

     // If no map is present, or the current map has been updated
    if(!this->_mapAnalyzer.ready() || (req->map.info.width > 0 && req->map.info.height > 0))
        this->setMap(req->map);

    // Check that repulsive forces in q_goal are null
    Vector4d fr_g;
    for(auto q : req->q){
        fr_g = _computeRepulsiveForce(q.position.x, q.position.y);
        if(fr_g[0] != 0 || fr_g[1] != 0){
            ROS_ERROR("Repulsive forces in one of goal configurations are not null. Planning is not possible");
            return;
        }
    }

    // Build path msg
    Trajectory trajectory;
    _path.header.stamp = ros::Time::now();
    _path.header.frame_id = "worldNED";

    // Plan all the segments
    double steadyT = DEFAULT_STEADY_TIME;
    for(int i=1; i < req->q.size(); ++i){
        // Select steady time
        if(i-1 < req->steadyTime.size())
            steadyT = req->steadyTime[i-1];
        else
            steadyT = DEFAULT_STEADY_TIME;

        _planSegment(req->q[i-1], req->q[i], steadyT, trajectory, _path);
    }

    trajectory.sampleTime = this->_sampleTime;

    if(_debugPath)
        this->_pathPub.publish(_path);
    this->_pub.publish(trajectory);

    _done = true;

    ROS_INFO("APPlanner_2D: path planning successfully completed.");
}


void APPlanner2D::run(){
    ros::Rate rate(_rate);
    while(ros::ok()){
        if(_done && _debugPath)
            _pathPub.publish(_path);
        rate.sleep();
        ros::spinOnce();
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner");
    APPlanner2D planner;
    planner.run();
    return 0;
}
