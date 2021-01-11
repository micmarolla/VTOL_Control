#include <string>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include "quad_control/PlanRequest.h"
#include "quad_control/Trajectory.h"
#include "quad_control/wind.h"

ros::Publisher pub;
quad_control::PlanRequest req;

bool ready = false;
bool pending = false;



void map_cb(nav_msgs::OccupancyGridConstPtr data){
    if(!ready){
        req.map = *data;
        ready = true;
    }
}


void traj_cb(quad_control::TrajectoryConstPtr data){

}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner_test");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);
    ros::Subscriber sub = nh.subscribe("/map", 0, &map_cb);
    ros::Subscriber subT = nh.subscribe("/trajectory", 0, &traj_cb);
    pub = nh.advertise<quad_control::PlanRequest>("/planRequest", 0, true);

    int n = nh.param<int>("n", 0);
    if(n < 2){
        ROS_ERROR("Number of points must be at least 2");
        return 1;
    }

    quad_control::UAVPose q;

    for(int i=0; i<n; ++i){
        q.position.x = nh.param<double>("x"+std::to_string(i), 0.0);
        q.position.y = nh.param<double>("y"+std::to_string(i), 0.0);
        q.position.z = nh.param<double>("z"+std::to_string(i), 0.0);
        q.yaw        = nh.param<double>("yaw"+std::to_string(i), 0.0);
        if(i > 0){
            double t = nh.param<double>("steadyTime"+std::to_string(i), 1.0);
            req.steadyTime.push_back(t);
        }
        req.q.push_back(q);
    }

    while(ros::ok()){
        if(ready && !pending){
            pending = true;
            pub.publish(req);
        }

        rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
