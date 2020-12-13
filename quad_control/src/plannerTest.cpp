#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "quad_control/APPlanner2D.h"
#include "APPlanner2D_Server.h"
#include <iostream>

nav_msgs::OccupancyGrid testMap;
bool ready = false;
bool done = false;


void map_cb(nav_msgs::OccupancyGridConstPtr data){
    if(!ready){
        testMap = *data;
        ROS_INFO("MAP READY!!!");
        ready = true;
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner_test");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    ros::ServiceClient client = nh.serviceClient<quad_control::APPlanner2D>("/planning_srv");
    ros::Subscriber sub = nh.subscribe("/map", 1, &map_cb);
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/plannedPath", 0);
    
    quad_control::APPlanner2D srv;
    srv.request.qs.position.x = srv.request.qs.position.y =
        srv.request.qs.position.z = 0;
    srv.request.qs.orientation.x = srv.request.qs.orientation.y =
        srv.request.qs.orientation.z = 0;
    srv.request.qs.orientation.w = 1;
    srv.request.qg.position.x = 5;
    srv.request.qg.position.y = 10;
    srv.request.qg.position.z = 3;
    srv.request.qg.orientation.x = srv.request.qg.orientation.y =
        srv.request.qg.orientation.z = 0;
    srv.request.qg.orientation.w = 1;

    nav_msgs::Path path;

    ROS_INFO("WAITING FOR MAP...");

    while(ros::ok()){
        if(ready){
            if(!done){
                srv.request.map = testMap;
                if(client.call(srv)){
                    cout << "PATH PLANNED RECEIVED! Size: " << srv.response.path.poses.size() << endl;
                    path = srv.response.path;
                    done = true;
                    auto last = srv.response.path.poses.back().pose.position;
                    cout << "Last position: " << last.x << ", " << last.y << ", " << last.z << endl;
                }
            }
            pub.publish(path);
        }
        rate.sleep();
        ros::spinOnce();    
    }

    return 0;
}
