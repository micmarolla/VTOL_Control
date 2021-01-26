#include "Manager.h"
#include "quad_control/TrajectoryPoint.h"

Manager::Manager() : _nh("~"){
    _rate      = _nh.param<double>("rate", 100.0);
    _toPlan    = _nh.param<bool>("planTrajectory", false);

    _mapSub  = _nh.subscribe("/map", 0, &Manager::_map_cb, this);
    _planPub = _nh.advertise<quad_control::PlanRequest>("/planRequest", 0, true);

    _planned = false;
    _trajPublishing = false;
    _currTrajPoint = 0;
    _trajPerc = 10;

    ROS_INFO("Manager initialized.");
}


void Manager::_map_cb(nav_msgs::OccupancyGridConstPtr data){
    if(!_mapReady){
        _req.map = *data;
        _mapReady = true;
    }
}


void Manager::requestPlanning(){
    if(!_toPlan || !_mapReady)
        return;

    quad_control::UAVPose q;

    double prevYaw = 0;
    int i = 0;
    while(_nh.hasParam("x"+std::to_string(i)) || _nh.hasParam("y"+std::to_string(i))
            || _nh.hasParam("z"+std::to_string(i)) || _nh.hasParam("yaw"+std::to_string(i))){
        q.position.x = _nh.param<double>("x"+std::to_string(i), 0.0);
        q.position.y = _nh.param<double>("y"+std::to_string(i), 0.0);
        q.position.z = _nh.param<double>("z"+std::to_string(i), 0.0);
        q.yaw        = _nh.param<double>("yaw"+std::to_string(i), prevYaw);
        prevYaw = q.yaw;
        if(i > 0){
            double t = _nh.param<double>("steadyTime"+std::to_string(i), 0.0);
            _req.steadyTime.push_back(t);
        }
        ++i;
        _req.q.push_back(q);
    }

    if(i < 1){
        ROS_ERROR("Number of points must be at least 2. Planning aborted.");
        _req.q.clear();
        return;
    }

    ROS_INFO("Requesting trajectory planning...");
    _planPub.publish(_req);
    _planned = true;
}


void Manager::run(){
    ros::Rate rate(_rate);
    while(ros::ok()){
        // Plan trajectory
        if(_toPlan && !_planned && _mapReady)
            requestPlanning();

        // Do something else?
        // ...

        rate.sleep();
        ros::spinOnce();
    }
}

int main(int argc, char **argv){
    ros::init(argc, argv, "manager");
    Manager manager;
    manager.run();
    return 0;
}
