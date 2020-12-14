#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include "quad_control/APPlanner2D.h"
#include "APPlanner2D_Server.h"

nav_msgs::OccupancyGrid testMap;
bool ready = false;
bool done = false;


void map_cb(nav_msgs::OccupancyGridConstPtr data){
    if(!ready){
        testMap = *data;
        ready = true;
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner_test");
    ros::NodeHandle nh("~");
    ros::Rate rate(1);
    ros::ServiceClient client = nh.serviceClient<quad_control::APPlanner2D>("/planning_srv");
    ros::Subscriber sub = nh.subscribe("/map", 1, &map_cb);
    ros::Publisher pub = nh.advertise<nav_msgs::Path>("/plannedPath", 0);
    

    double startX = nh.param<double>("startX", 0.0);
    double startY = nh.param<double>("startY", 0.0);
    double startZ = nh.param<double>("startZ", 0.0);
    double endX   = nh.param<double>("endX",   1.0);
    double endY   = nh.param<double>("endY",   1.0);
    double endZ   = nh.param<double>("endZ",   1.0);


    quad_control::APPlanner2D srv;
    srv.request.qs.position.x = startX;
    srv.request.qs.position.y = startY;
    srv.request.qs.position.z = startZ;

    srv.request.qs.orientation.x = srv.request.qs.orientation.y =
        srv.request.qs.orientation.z = 0;
    srv.request.qs.orientation.w = 1;

    srv.request.qg.position.x = endX;
    srv.request.qg.position.y = endY;
    srv.request.qg.position.z = endZ;

    srv.request.qg.orientation.x = srv.request.qg.orientation.y =
        srv.request.qg.orientation.z = 0;
    srv.request.qg.orientation.w = 1;

    nav_msgs::Path path;

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
