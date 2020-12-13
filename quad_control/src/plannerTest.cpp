#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "quad_control/APPlanner2D.h"
#include "APPlanner2D_Server.h"

nav_msgs::OccupancyGrid testMap;
bool ready = false;
bool done = false;


void map_cb(nav_msgs::OccupancyGridConstPtr data){
    testMap = *data;
    ROS_INFO("MAP READY!!!");
    ready = true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner_test");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    ros::ServiceClient client = nh.serviceClient<quad_control::APPlanner2D>("/planning_srv");
    ros::Subscriber sub = nh.subscribe("/map", 0, &map_cb);
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/plannedPath", 0);
    
    quad_control::APPlanner2D srv;
    geometry_msgs::Pose qs, qg;
    qs.position.x = qs.position.y = qs.position.z = 0;
    qg.position.x = 5; qg.position.y = 10; qg.position.z = 3;

    nav_msgs::Path path;

    ROS_INFO("WAITING FOR MAP...");

    while(ros::ok()){
        if(ready){
            if(!done){
                srv.request.map = testMap;
                if(client.call(srv)){
                    path = srv.response.path;
                    done = true;
                }
            }
            pub.publish(path);
        }
        rate.sleep();
        ros::spinOnce();    
    }

    return 0;
}
