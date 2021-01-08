#include "APPlanner2D.h"

#include <cmath>
#include <vector>
#include <queue>

#include <geometry_msgs/Accel.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

using namespace quad_control;
using namespace Eigen;

APPlanner2D::APPlanner2D() : _nh("~"){
    // Retrieve params
    _rate           = _nh.param<double>("rate", 1.0);
    _ka             = _nh.param<double>("ka", 1.0);
    _kr             = _nh.param<double>("kr", 1.0);
    _qdiffMin       = _nh.param<double>("qDiffMin", 0.0);
    _qdiffMax       = _nh.param<double>("qDiffMax", 0.0);
    _gamma          = _nh.param<double>("gamma", 2.0);
    _p_eps          = _nh.param<double>("p_eps", 0.001);
    _o_eps          = _nh.param<double>("o_eps", 0.001);
    _debugPath      = _nh.param<bool>  ("debugPath", true);
    _navVel         = _nh.param<double>("navVel", 1.0);
    _maxVertAcc     = _nh.param<double>("maxVerticalAcc", 5.0);
    _navFuncRadius  = _nh.param<double>("navFuncRadius", 3.0);
    _goalDistAvg    = _nh.param<double>("goalDistAvg", 0.1);
    _goalDistMin    = _nh.param<double>("goalDistMin", 0.05);

    if(_nh.hasParam("sampleTime")){
        _sampleMin = _sampleMax = _sampleAvg
                = _nh.param<double>("sampleTime", 0.01);
    }else{
        _sampleMin = _nh.param<double>("sampleTimeMin", 0.001);
        _sampleMax = _nh.param<double>("sampleTimeMax", 0.01);
        _sampleAvg = (_sampleMin + _sampleMax) / 2.0;
    }

    // Eta
    if(_nh.hasParam("eta")){
        double eta = _nh.param<double>("eta", 1.0);
        _etaObst = _etaWall = eta;
    }else{
        _etaWall = _nh.param<double>("etaWall", 1.0);
        _etaObst = _nh.param<double>("etaObst", 1.0);
    }
    _etaMax = (_etaObst > _etaWall) ? _etaObst : _etaWall;

    _currMinDist2 = 0;
    _done = false;
    _obstacleNearby = false;

    _sub = _nh.subscribe("/planRequest", 0, &APPlanner2D::plan, this);
    _pub = _nh.advertise<Trajectory>("/trajectory", 0, true);

    if(_debugPath)
        _pathPub = _nh.advertise<nav_msgs::Path>("/plannedPath", 0, false);
}


void APPlanner2D::setMap(nav_msgs::OccupancyGrid &map){
    this->_mapInfo = map.info;
    this->_mapAnalyzer.setMap(map);
    this->_mapAnalyzer.scan();
}


Vector4d APPlanner2D::_computeError(UAVPose q, UAVPose qd){
    return Vector4d (qd.position.x - q.position.x, qd.position.y - q.position.y,
            qd.position.z - q.position.z, qd.yaw - q.yaw);
}


UAVPose APPlanner2D::_eulerIntegration(UAVPose q, Vector4d ft, double sampleTime){
    // Cap total force
    if (_qdiffMax > 0){
        double diff = _qdiffMax;
        if(sampleTime == _sampleMin)
            diff = _qdiffMin;
        double fMax = diff / sampleTime;
        if (ft.norm() > fMax)
            ft = ft / ft.norm() * fMax;
    }

    // Compute q
    q.position.x = q.position.x + sampleTime * ft[0];
    q.position.y = q.position.y + sampleTime * ft[1];
    q.position.z = q.position.z + sampleTime * ft[2];
    q.yaw = q.yaw + sampleTime * ft[3];

    return q;
}


Vector4d APPlanner2D::_computeForce(UAVPose q, Vector4d e){
    Vector4d fa, fr;

    // Attractive potentials
    if (e.norm() <= 1)
        fa = this->_ka * e;                 // Paraboloid
    else
        fa = this->_ka * e / e.norm();      // Conical

    // Repulsive potentials
    fr = _computeRepulsiveForce(q.position.x, q.position.y);
    _obstacleNearby = fr.norm() != 0;

    return fa + fr;
}


Vector4d APPlanner2D::_computeRepulsiveForce(double rx, double ry){
    Vector4d fr (0,0,0,0);
    Vector2d fri;
    double fri_mod = 0;

    // Retrieve robot coordinates in the map cell reference
    int rxCell = (rx - this->_mapInfo.origin.position.x) / this->_mapInfo.resolution;
    int ryCell = (ry - this->_mapInfo.origin.position.y) / this->_mapInfo.resolution;

    // For each chunk, considering the minimum-distance point from the robot...
    vector<Chunk*> obstacles = this->_mapAnalyzer.getObjAtMinDist(rxCell, ryCell);
    bool firstObst = true;
    double eta = 0;
    for (auto obj : obstacles){
        obj->dist2 *= pow(this->_mapInfo.resolution, 2); // convert to [meters^2]

        /* Distinguish between wall and obstacle:
        this works because the first chunk is always the wall */
        if (firstObst){
            eta = _etaWall;
            _currMinDist2 = obj->dist2;
            firstObst = false;
        }else{
            eta = _etaObst;
            _currMinDist2 = (obj->dist2 < _currMinDist2) ? obj->dist2 : _currMinDist2;
        }

        if (obj->dist2 > pow(eta,2))
            continue;

        // ... compute repulsive force
        fri_mod = (_kr / obj->dist2) * pow(1/sqrt(obj->dist2) - 1/eta, _gamma - 1);
        fri = fri_mod * Vector2d(rxCell - obj->x, ryCell - obj->y).normalized();

        fr[0] += fri[0];
        fr[1] += fri[1];
    }

    return fr;
}


int APPlanner2D::_findNavSubGoal(int subOx, int subOy, int subW, int subH, int qgx, int qgy){
    if(qgx > subOx && qgx <= subOx + subW && qgy > subH && qgy <= subOy + subH)
        return qgx * _mapInfo.width + qgy;

    int currX = subOx;
    int currY = subOy;
    int minX = currX;
    int minY = currY;

    bool clockwise = true;
    double minDist2 = pow(currX-qgx,2) + pow(currY-qgy,2);
    double tempDist2;

    bool done = false;

    while(!done){
        if(_mapAnalyzer.cellValue(currX, currY) < 50){
            tempDist2 = pow(currX-qgx,2) + pow(currY-qgy,2);
            if(tempDist2 <= minDist2){
                minDist2 = tempDist2;
                minX = currX;
                minY = currY;
            }
            else{
                if(clockwise){
                    clockwise = false;
                    currX = subOx;
                    currY = subOy;
                }
                else
                    done = true;
            }
        }

        // Retrieve the new cell
        if(clockwise){
            if(currY == subOy && currX < subOx + subH - 1)  // left side
                ++currX;
            else if(currX == subOx + subH -1 && currY < subOy + subW - 1) // top
                ++currY;
            else if(currY == subOy + subW - 1&& currX > subOx) // right
                --currX;
            else if(currX == subOx && currY > subOy) // bottom
                --currY;
        }else{
            if(currY == subOy && currX > subOx)  // left side
                --currX;
            else if(currX == subOx + subH - 1 && currY > subOy) // top
                --currY;
            else if(currY == subOy + subW - 1 && currX < subOx + subH - 1) // right
                ++currX;
            else if(currX == subOx && currY < subOy + subW - 1) // bottom
                ++currY;
        }
    }

    return ((minX-subOx) * subW) + (minY-subOy);
}


UAVPose APPlanner2D::_handleLocalMinima(UAVPose q, UAVPose qg,
        Trajectory& trajectory, nav_msgs::Path& path){
    // Robot and goal coordinates in map frame
    int qx = (q.position.x - _mapInfo.origin.position.x) / _mapInfo.resolution;
    int qy = (q.position.y - _mapInfo.origin.position.y) / _mapInfo.resolution;
    int qgx = (qg.position.x - _mapInfo.origin.position.x) / _mapInfo.resolution;
    int qgy = (qg.position.y - _mapInfo.origin.position.y) / _mapInfo.resolution;

    // Compute submap size and robot position in submap
    int subW, subH;
    int halfSubW, halfSubH;
    int subX, subY;
    int subOx, subOy;   // Submap origin in map frame
    int exceeding = 0;  // Number of submap cells outside the actual map borders

    // Robot is always at the center...
    subW = subH = 2 * _navFuncRadius / _mapInfo.resolution + 1;
    halfSubW = halfSubH = (subW - 1) / 2;
    subX = subY = ceil(subW/2);
    subOx = qx - halfSubH;
    subOy = qy - halfSubW;

    // ... unless it's near the map border

    // Left
    if(qy - halfSubW < 0){
        exceeding = halfSubW - qy;
        subW -= exceeding;
        subY = qy;
        subOy += exceeding;
    }

    // Bottom
    if(qx - halfSubH < 0){
        exceeding = halfSubH - qx;
        subH -= exceeding;
        subX = qx;
        subOx += exceeding;
    }

    // Right
    if(qy + halfSubW >= _mapInfo.width){
        exceeding = qy + halfSubW - _mapInfo.width;
        subW -= exceeding;
    }

    // Top
    if(qx + halfSubH >= _mapInfo.height){
        exceeding = qx + halfSubH - _mapInfo.height;
        subH -= exceeding;
    }

    // Construct submap and navigation function
    ROS_INFO_STREAM("Generating submap...");
    int8_t* submap = _mapAnalyzer.generateSubmap(qx, qy, subW, subH);
    NavigationFunc nf;
    nf.setMap(submap, subW, subH);

    // Find the submap point nearest to the actual goal
    ROS_INFO_STREAM("Finding subgoal...");
    int subGoalX, subGoalY;
    int subGoal = _findNavSubGoal(subOx, subOy, subW, subH, qgx, qgy);
    subGoalX = subGoal / subW;
    subGoalY = subGoal % subW;

    // Build navigation function (lazy build, i.e., q is specified)
    ROS_INFO_STREAM("Building navigation function...");
    ROS_INFO_STREAM("    Map size: " << subW << ", " << subH);
    ROS_INFO_STREAM("    SubRobot: " << subX << ", " << subY);
    ROS_INFO_STREAM("    Subgoal: "  << subGoalX << ", " << subGoalY);

    const int* nav = nf.scan(subGoalX, subGoalY, subX, subY);
    std::queue<int>* nfPath = nf.getPath();

    // Add the retrieved path to the trajectory
    int next = 0, x = 0, y = 0;

    UAVPose p;  // Keep the same z and yaw as starting pose q
    p.position.z = q.position.z;
    p.yaw = q.yaw;

    // Velocity, acceleration
    geometry_msgs::Accel v;
    v.angular.x = v.angular.y = v.angular.z = v.linear.z = 0;
    Vector2d vel, velPrev;
    bool first = true;

    ROS_INFO_STREAM("Building path (" << nfPath->size() << " points)...");
    while(!nfPath->empty()){
        next = nfPath->front();
        nfPath->pop();

        // Cell coordinate in map frame
        x = next / subW + subOx;
        y = next % subW + subOy;

        // Create the UAV pose, velocity and acceleration
        p.position.x = x * _mapInfo.resolution + _mapInfo.origin.position.x;
        p.position.y = y * _mapInfo.resolution + _mapInfo.origin.position.y;
        trajectory.t.push_back(_sampleAvg);
        trajectory.p.push_back(p);

        // Velocity
        vel << p.position.x, p.position.y;
        vel = vel.normalized() * _navVel;
        v.linear.x = vel[0];
        v.linear.y = vel[1];
        trajectory.v.push_back(v);

        // Acceleration as simple numerical derivation
        if(first)
            first = false;
        else{
            v.linear.x = (vel[0] - velPrev[0]) / _sampleAvg;
            v.linear.y = (vel[1] - velPrev[1]) / _sampleAvg;
            trajectory.a.push_back(v);
        }
        velPrev = vel;

        // Publish debug path
        if(_debugPath){
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "worldNED";
            pose.pose.position = p.position;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(p.yaw);
            path.poses.push_back(pose);

            this->_pathPub.publish(_path);
        }
    }

    // Append final entries with null velocity and acceleration, and same position
    v.linear.x = v.linear.y = 0;
    trajectory.p.push_back(trajectory.p.back());
    trajectory.v.push_back(v);
    trajectory.a.push_back(v);  // Twice, for having the same number of points
    trajectory.a.push_back(v);

    return p;
}


void APPlanner2D::_planSegment(UAVPose qs, UAVPose qg, double steadyTime,
        Trajectory& trajectory, nav_msgs::Path& path){

    UAVPose q = qs;
    geometry_msgs::Accel v;
    v.angular.x = v.angular.y = 0;

    Vector4d err, ft, ftPrev;
    Vector3d goalDist;
    bool done = false, first = true;

    UAVPose prevQ = q;
    int prevCounter = 0;

    while (!done){
        // Compute error, force, integrate and add to path
        err = _computeError(q, qg);
        ft  = _computeForce(q, err);

        goalDist << q.position.x - qg.position.x, q.position.y - qg.position.y,
            q.position.z - qg.position.z;

        // Sample time
        double sample;
        if (_currMinDist2 > 2*_etaMax || goalDist.norm() > _goalDistAvg)
            sample = _sampleMax;        // Far from obstacles
        else if (_currMinDist2 > _etaMax || goalDist.norm() > _goalDistMin)
            sample = _sampleAvg;        // Mid-way
        else
            sample = _sampleMin;        // Near obstacles
        trajectory.t.push_back(sample);

        q = _eulerIntegration(q, ft, sample);

        // Accelerations: simple numerical derivation
        if(first)
            first = false;
        else{
            v.linear.x  = (ft[0]-ftPrev[0]) / sample;
            v.linear.y  = (ft[1]-ftPrev[1]) / sample;
            v.linear.z  = (ft[2]-ftPrev[2]) / sample;
            v.linear.z = gAccCap(v.linear.z);
            v.angular.z = (ft[3]-ftPrev[3]) / sample;
            trajectory.a.push_back(v);
        }
        ftPrev = ft;

        // Velocities
        v.linear.x = ft[0]; v.linear.y = ft[1]; v.linear.z = ft[2];
        v.angular.z = ft[3];
        trajectory.v.push_back(v);

        // Positions
        trajectory.p.push_back(q);

        // Debug path
        if(_debugPath){
            geometry_msgs::PoseStamped pose;
            pose.header.stamp = ros::Time::now();
            pose.header.frame_id = "worldNED";
            pose.pose.position = q.position;
            pose.pose.orientation = tf::createQuaternionMsgFromYaw(q.yaw);
            path.poses.push_back(pose);

            this->_pathPub.publish(_path);
        }

        // Check if goal has been reached
        done = (Vector3d(err[0],err[1],err[2]).norm() <= this->_p_eps && err[3] <= this->_o_eps);
        if(done)
            break;

        // Check if the trajectory is stuck in a local minima
        if (prevCounter < 50)   ++prevCounter;
        else if (_obstacleNearby && q.position.z == prevQ.position.z){
            prevCounter = 0;
            Vector2d disp (q.position.x - prevQ.position.x, q.position.y - prevQ.position.y);
            prevQ = q;
            if(disp.norm() < 1e-2){
                ROS_INFO("Stuck in a local minimum!");
                q = _handleLocalMinima(q, qg, trajectory, path);
            }
        }

    }

    // Append final entries with null velocity and acceleration, and same position
    for(int i=0; i < ceil(steadyTime / _sampleMax); ++i){
        v.linear.x = v.linear.y = v.linear.z = 0;
        v.angular.x = v.angular.y = v.angular.z = 0;
        trajectory.p.push_back(trajectory.p.back());
        trajectory.v.push_back(v);
        trajectory.a.push_back(v);  // Twice, for having the same number of points
        trajectory.a.push_back(v);
    }
}


void APPlanner2D::plan(PlanRequestPtr req){
    ROS_INFO("APPlanner_2D: path planning requested. Planning...");

    // Check number of points in the trajectory
    if(req->q.size() <= 1){
        ROS_ERROR("Number of points in PlanRequest must be at least 2.");
        return;
    }

     // If no map is present, or the current map has been updated
    if(!this->_mapAnalyzer.ready() || (req->map.info.width > 0 && req->map.info.height > 0))
        this->setMap(req->map);

    // Check that repulsive forces in q_goal are null
    Vector4d fr_g;
    int currPoint = 0;
    for(auto q : req->q){
        fr_g = _computeRepulsiveForce(q.position.x, q.position.y);
        if(fr_g[0] != 0 || fr_g[1] != 0){
            ROS_ERROR_STREAM("Repulsive forces in configurations " <<
                currPoint << " are not null. Planning is not possible");
            ROS_ERROR_STREAM("Distance from the nearest obstacle: " << sqrt(_currMinDist2));
            return;
        }
        ++currPoint;
    }

    // Wait for transform between world and worldNED
    if(_debugPath){
        tf::TransformListener _tf;
        try{
            _tf.waitForTransform("/world", "/worldNED", ros::Time::now(),
                ros::Duration(1.0));
        } catch(tf::TransformException &ex){
            ROS_ERROR("%s", ex.what());
            _debugPath = false;
        }
    }

    // Build path msg
    Trajectory trajectory;
    _path.header.stamp = ros::Time::now();
    _path.header.frame_id = "worldNED";

    // Plan all the segments
    double steadyT = 0;
    for(int i=1; i < req->q.size(); ++i){
        // Select steady time
        if(i-1 < req->steadyTime.size())
            steadyT = req->steadyTime[i-1];
        else
            steadyT = 0;

        _planSegment(req->q[i-1], req->q[i], steadyT, trajectory, _path);
    }

    // Publish trajectory
    if(_debugPath)
        this->_pathPub.publish(_path);
    this->_pub.publish(trajectory);

    _done = true;

    ROS_INFO("APPlanner_2D: path planning successfully completed.");
}


void APPlanner2D::run(){
    ros::Rate rate(_rate);
    while(ros::ok()){
        if(_done && _debugPath)
            _pathPub.publish(_path);
        rate.sleep();
        ros::spinOnce();
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "2d_planner");
    APPlanner2D planner;
    planner.run();
    return 0;
}
