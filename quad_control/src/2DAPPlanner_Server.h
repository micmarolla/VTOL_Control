/*
 * 2DAPPlanner provides a service that realizes path planning via artificial
 * potentials method, for a 2D known environment.
 * The planning is realized considering the virtual force acting on the robot
 * as a velocity, and the integration is realized through Euler method.
 * It is assumed that each C-Obstacle has the same range of influence, and all
 * the repulsive force gains k_(r,i) are the same.
 * Corresponding .srv file is 2DAPPlanner.srv
 */

#include "ros/ros.h"
#include "quad_control/2DAPPlanner.h"
#include "geometry_msgs/Pose.h"

class 2DAPPlanner_Server{

    private:
        ros::NodeHandle _nh;
        ros::ServiceServer _server;
        double _ka, _kr;    // attractive and repulsive forces gain
        double _eta;        // range of influence
        double _gamma;      // param for repulsive forces, gamma = {2, 3}
        double _p_eps;      // if position error <= eps, it is assumed error = 0
        double _o_eps;      // if orientation error <= eps, it is assumed error = 0
        double _sampleTime;
        

        // Compute next configuration using Euler integration
        geometry_msgs::Pose _eulerIntegration(geometry_msgs::Pose q);


    public:
        2DAPPlanner_Server();

        /*
         * Compute the virtual forces applied on the robot, via the artificial
         * potentials method.
         * Parameters:
         *  - q: current robot configuration
         *  - qg: goal configuration
         */
        double computeForce(geometry_msgs::Pose q, geometry_msgs::Pose qg);

        /*
         * Plan the trajectory. Return true if the planning was successfull;
         * otherwise, print an error message and return false.
         */
        bool plan(quad_control::2DAPPlanner::Request &req, quad_control::2DAPPlanner::Response &res);

};
