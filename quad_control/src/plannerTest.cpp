#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "quad_control/PlanRequest.h"
#include "quad_control/Trajectory.h"

ros::Publisher pub;
quad_control::PlanRequest req;
bool ready = false;


void map_cb(nav_msgs::OccupancyGridConstPtr data){
    if(!ready){
        ready = true;
        req.map = *data;
        pub.publish(req);
    }
}


void traj_cb(quad_control::TrajectoryConstPtr data){
    ROS_INFO_STREAM("Positions: " << data->p.size());
    ROS_INFO_STREAM("Velocities: " << data->v.size());
    ROS_INFO_STREAM("Accelerations: " << data->a.size());

    ROS_INFO_STREAM("First pos: " << data->p[0].position.x << ", "
        << data->p[0].position.y << ", " << data->p[0].position.z);

    ROS_INFO_STREAM("First vel: " << data->v[0].linear.x << ", "
        << data->v[0].linear.y << ", " << data->v[0].linear.z);

    ROS_INFO_STREAM("First accel: " << data->a[0].linear.x << ", "
        << data->a[0].linear.y << ", " << data->a[0].linear.z);
}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner_test");
    ros::NodeHandle nh("~");
    ros::Rate rate(1);
    ros::Subscriber sub = nh.subscribe("/map", 0, &map_cb);
    pub = nh.advertise<quad_control::PlanRequest>("/planRequest", 0, true);

    double startX = nh.param<double>("startX", 0.0);
    double startY = nh.param<double>("startY", 0.0);
    double startZ = nh.param<double>("startZ", 0.0);
    double endX   = nh.param<double>("endX",   1.0);
    double endY   = nh.param<double>("endY",   1.0);
    double endZ   = nh.param<double>("endZ",   1.0);

    req.qs.position.x = startX;
    req.qs.position.y = startY;
    req.qs.position.z = startZ;
    req.qs.yaw = 0;

    req.qg.position.x = endX;
    req.qg.position.y = endY;
    req.qg.position.z = endZ;
    req.qg.yaw = 0;

    while(ros::ok()){
        rate.sleep();
        ros::spinOnce(); 
    }

    return 0;
}
