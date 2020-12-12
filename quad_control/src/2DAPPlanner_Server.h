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
#include <Eigen/Dense>

typedef Eigen::Matrix<double, 6, 1> Vector6d;

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
        geometry_msgs::Pose _eulerIntegration(geometry_msgs::Pose q, Vector6d ft);
        
        // Compute position and orientation (RPY) error
        Vector6d _computeError(geometry_msgs::Pose q, geometry_msgs::Pose qd);

        /*
         * Compute the virtual forces applied on the robot, via the artificial
         * potentials method.
         * Parameters:
         *  - e: error vector
         */
        Vector6d _computeForce(Vector6d e);


    public:
        2DAPPlanner_Server();

        /*
         * Plan the trajectory. Return true if the planning was successfull;
         * otherwise, print an error message and return false.
         */
        bool plan(quad_control::2DAPPlanner::Request &req, quad_control::2DAPPlanner::Response &res);

};
