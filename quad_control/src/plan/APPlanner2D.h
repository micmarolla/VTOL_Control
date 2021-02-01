#ifndef _APPLANNER2D_
#define _APPLANNER2D_

#include <queue>

#include <ros/ros.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/MapMetaData.h>
#include <Eigen/Dense>

#include "quad_control/PlanRequest.h"
#include "quad_control/Trajectory.h"
#include "quad_control/UAVPose.h"
#include "MapAnalyzer.h"
#include "NavigationFunc.h"

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
     * Find the point in the submap closest to the goal in the map.
     * If the goal is inside the submap, the goal itself is returned.
     * Otherwise, the point placed on the border of the submap and with the
     * minimum distance to the actual goal is returned.
     * Parameters:
     *  - subOx, subOy, subW, subH: origin and size of the submap wrt map
     *  - qgx, qgy: goal cell coordinates in map frame
     */
    int _findNavSubGoal(int subOx, int subOy, int subW, int subH, int qgx, int qgy);

    /*
     * Interpolate the discrete points retrieved using the navigation function,
     * generating a smooth trajectory. This is appended to the current trajectory.
     * Parameters:
     *  - q: current UAV pose
     *  - qx, qy: UAV coordinates in the map frame
     *  - nfPath: the path given by the navigation function
     *  - subOx, subOy, subW, subH: origin and size of the submap wrt map
     *  - trajectory, path: trajectory and path planned objects, to which the
     *      nav-func path will be appended.
     * Return the last UAV pose in the trajectory.
     */
    UAVPose _interpNavTraj(UAVPose q, int qx, int qy, std::queue<int>* nfPath,
        int subOx, int subOy, int subW, int subH,
        Trajectory& trajectory, nav_msgs::Path& path);

    /*
     * Handle the trajectory planning when stuck in a local minimum.
     * In detail, this generates a submap around the current position,
     * find the goal to reach in this submap, and build the navigation function.
     * Parameters:
     *  - q: current UAV pose
     *  - qg: goal pose
     *  - trajectory, path: trajectory and path planned objects, to which the
     *      nav-func path will be appended.
     * Return the last UAV pose in the trajectory.
     */
    UAVPose _handleLocalMinima(UAVPose q, UAVPose qg, Trajectory& trajectory,
        nav_msgs::Path& path);

    /*
     * Plan a trajectory along the segment between qs and qg.
     * The trajectory stays in qg for a time equal to steadyTime [seconds].
     * The output is appended to the msg trajectory and path (the latter used
     * for debug).
     */
    void _planSegment(UAVPose qs, UAVPose qg, double steadyTime,
        Trajectory& trajectory, nav_msgs::Path& path);


    /* Limit the maximum acceleration along the z-axis. */
    double gAccCap(double a){
        if(a > _maxVertAcc) return _maxVertAcc;
        return a;
    }


private:
    ros::NodeHandle _nh;
    ros::Subscriber _sub;
    ros::Publisher _pathPub, _pathPointsPub, _pub;

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
    double _maxVel_p, _maxVel_o;    // Maximum linear and angular velocity

    bool _showPath;     // If true, publish nav_msgs::Path debug msg
    bool _showPathPoints;   // If true, publish path segments' extremes
    bool _done;         // True if trajectory has been planned and published

    bool _obstacleNearby;

    double _navFuncRadius;  // Radius of the submap considered for navfunc
    double _navVel;         // Velocity while in navigation map
    double _navEta;         // Radius of obstacle isotropic expansion for navfunc
    double _navSample;      // Sample time used for navfunc
    int _navRatio;          // Navfunc / grid cell resolution ratio
    double _navErrTolerance;        // Error tolerance when building nav path
    double _navMaxFt, _navMaxDisp;  // Max force and displacement to recognize a local minimum

    nav_msgs::Path _path;
    geometry_msgs::PoseArray _pathPoints;

    nav_msgs::MapMetaData _mapInfo;
    MapAnalyzer _mapAnalyzer;
};

#endif
