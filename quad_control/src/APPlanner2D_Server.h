/*
 * APPlanner2D provides a service that realizes path planning via artificial
 * potentials method, for a 2D known environment.
 * The planning is realized considering the virtual force acting on the robot
 * as a velocity, and the integration is realized through Euler method.
 * It is assumed that each C-Obstacle has the same range of influence, and all
 * the repulsive force gains k_(r,i) are the same.
 * Corresponding .srv file is APPlanner2D.srv
 */

#ifndef _APPLANNER2D_SERVER_
#define _APPLANNER2D_SERVER_

#include "ros/ros.h"
#include "quad_control/APPlanner2D.h"
#include "geometry_msgs/Pose.h"
#include "nav_msgs/OccupancyGrid.h"
#include <Eigen/Dense>
#include "MapAnalyzer.h"

using namespace std;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

class APPlanner2D_Server{

    private:
        ros::NodeHandle _nh;
        ros::ServiceServer _server;
        double _ka, _kr;    // attractive and repulsive forces gain
        double _eta;        // range of influence
        double _gamma;      // param for repulsive forces, gamma = {2, 3}
        double _p_eps;      // if position error <= eps, it is assumed error = 0
        double _o_eps;      // if orientation error <= eps, it is assumed error = 0
        double _sampleTime;
        nav_msgs::OccupancyGrid _map;
        bool _mapReady;
        MapAnalyzer _mapAnalyzer;

        
        // Compute position and orientation (RPY) error
        Vector6d _computeError(geometry_msgs::Pose q, geometry_msgs::Pose qd);

        // Compute next configuration using Euler integration
        geometry_msgs::Pose _eulerIntegration(geometry_msgs::Pose q, Vector6d ft);

        /*
         * Compute the virtual forces applied on the robot, via the artificial
         * potentials method.
         */
        Vector6d _computeForce(geometry_msgs::Pose q, Vector6d e);
        
        //
        std::shared_ptr<int8_t[]> _getNeighbourhood(nav_msgs::OccupancyGrid &grid, Eigen::Vector3d pos, int &w, int &h, int &x, int &y);
        
        //
        Vector6d _computeRepulsiveForce(/*shared_ptr<int8_t[]> submap, int w, int h,*/ int rx, int ry);


    public:
        APPlanner2D_Server();

        void setMap(nav_msgs::OccupancyGrid &map);

        /*
         * Plan the trajectory. Return true if the planning was successfull;
         * otherwise, print an error message and return false.
         */
        bool plan(quad_control::APPlanner2D::Request &req, quad_control::APPlanner2D::Response &res);

};

#endif
