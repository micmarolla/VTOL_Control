#ifndef _APPLANNER2D_
#define _APPLANNER2D_

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <Eigen/Dense>

#include "quad_control/PlanRequest.h"
#include "quad_control/Trajectory.h"
#include "quad_control/UAVPose.h"
#include "MapAnalyzer.h"

using namespace quad_control;
using namespace Eigen;

/*
 * APPlanner2D realizes path planning via artificial potentials method, for a 2D
 * known environment. The planning is realized considering the virtual force
 * acting on the robot as a velocity, and the integration is realized through
 * Euler method.
 * It is assumed that each C-Obstacle has the same range of influence, and all
 * the repulsive force gains k_(r,i) are the same. A distinction is possible
 * between the outer walls and other obstacles.
 * Notice that the planning is made with respect to the world ENU frame.
 */
class APPlanner2D{

public:
    APPlanner2D();

    /*
     * Analyze the map and detect obstacles. It can be done only once for
     * multiple planning instances on the same map.
     */
    void setMap(nav_msgs::OccupancyGrid &map);

    /*
     * Plan the trajectory.
     * After the planning is made for the first time, the map is stored
     * You can use again the same map, setting to 0 the width or height in
     * req.map.info. In this way, the map is not analyzed again, and execution
     * time is spared.
     */
    void plan(PlanRequestPtr req);

    /* Main loop */
    void run();


private:

    /*
     * Compute position and orientation error, between actual and desired pose.
     * The orientation error is computed considering RPY angles.
     */
    Vector4d _computeError(UAVPose q, UAVPose qd);

    /*
     * Compute next configuration using Euler integration: qnext = q + ft*T.
     * Parameters:
     *  - q: robot pose
     *  - ft: total force acting on the robot
     *  - sampleTime: current sample time
     */
    UAVPose _eulerIntegration(UAVPose q, Vector4d ft, double sampleTime);

    /*
     * Compute the virtual forces applied on the robot, via the artificial
     * potentials method.
     * Parameters:
     *  - q: robot pose
     *  - e: error
     */
    Vector4d _computeForce(UAVPose q, Vector4d e);

    /*
     * Compute repulsive forces acting on the robot.
     * Parameters:
     *  - rx, ry: robot position (in meters)
     */
    Vector4d _computeRepulsiveForce(double rx, double ry);

    /*
     * Plan a trajectory along the segment between qs and qg.
     * The trajectory stays in qg for a time equal to steadyTime [seconds].
     * The output is appended to the msg trajectory and path (the latter used
     * for debug).
     */
    void _planSegment(UAVPose qs, UAVPose qg, double steadyTime,
        Trajectory& trajectory, nav_msgs::Path& path);


private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pathPub, _pub;

    double _rate;

    double _ka, _kr;    // Attractive and repulsive forces gain
    double _gamma;      // Used for repulsive forces computation, can be 2 or 3

    double _sampleMin, _sampleMax, _sampleAvg;      // Trajectory sample times

    double _etaObst, _etaWall, _etaMax;    // Range of influence
    double _currMinDist2;       // Minimum distance from robot to an obstacle
    double _p_eps;      // if position error <= eps, it is assumed error = 0
    double _o_eps;      // if orientation error <= eps, it is assumed error = 0
    double _qdiffMin, _qdiffMax;    // Limits on q displacement for each step
    double _goalDistMin, _goalDistAvg;  // Distance from goal limits that activate displacement capping
    double _maxVertAcc; // Maximum vertical acceleration

    bool _debugPath;    // If true, publish nav_msgs::Path debug msg
    bool _done;         // True if trajectory has been planned and published

    nav_msgs::Path _path;

    nav_msgs::MapMetaData _mapInfo;
    MapAnalyzer _mapAnalyzer;
};

#endif
