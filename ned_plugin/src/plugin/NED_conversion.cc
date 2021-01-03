#include <ros/ros.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/Plugin.hh>
#include "geometry_msgs/Wrench.h"
#include "nav_msgs/Odometry.h"
#include "mav_msgs/Actuators.h"
#include <eigen3/Eigen/Dense>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <string>

using namespace Eigen;
using namespace std;

namespace gazebo
{
  /*
   * NEDPlugin is a Gazebo world plugin that converts:
   *  - odometry information, into NED frame
   *  - wrench command, expressed in NED frame, into motor speed commands.
   * NB: it uses GROUND_TRUTH odometry data.
   */
  class NEDPlugin : public WorldPlugin
  {

  private:
     ros::NodeHandle* _nh;
     ros::Publisher _odomNED_pub, _commandNED_pub;
     ros::Subscriber _odom_sub, _command_sub;
     Matrix4d _G;


  public:
   /* Converts odometry data into NED frame. */
   void odom_cb( nav_msgs::OdometryConstPtr );
   /* Convert wrench commands into propellers velocities, and apply them to the UAV */
   void command_cb( geometry_msgs::WrenchConstPtr );
   /* Ensure that each squared propeller velocities is >= 0 */
   void correctW(Vector4d & w2);

   /* Init plugin */
   void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
		_nh = new ros::NodeHandle();

    ROS_INFO("NED conversions plugin started!");


    string odomTopic = "/hummingbird/ground_truth/odometry";
    bool useGroundTruth = _nh->param<bool>("/hummingbird/useGroundTruthOdom", true);
    if(!useGroundTruth)
        odomTopic = "/hummingbird/odometry_sensor1/odometry";

    _odom_sub = _nh->subscribe(odomTopic, 0, &NEDPlugin::odom_cb, this);
    _odomNED_pub = _nh->advertise< nav_msgs::Odometry>("/hummingbird/odometryNED", 0);
    _commandNED_pub = _nh->advertise< mav_msgs::Actuators>("/hummingbird/command/motor_speed", 0);
    _command_sub = _nh->subscribe("/hummingbird/command/wrenchNED", 0, &NEDPlugin::command_cb, this);

    double _l = 0.17; //meters
    double _c_T = 8.06428e-06;
    double _c_a = -0.000001;

    _G(0,0) = _c_T;    _G(0,1) = _c_T;    _G(0,2) = _c_T; _G(0,3) = _c_T;
    _G(1,0) = 0;       _G(1,1) = _l*_c_T; _G(1,2) = 0;    _G(1,3) = -_l*_c_T;
    _G(2,0) = -_l*_c_T; _G(2,1) = 0;       _G(2,2) = _l*_c_T; _G(2,3) = 0;
    _G(3,0) = -_c_a;    _G(3,1) = _c_a;    _G(3,2) = -_c_a; _G(3,3) = _c_a;

    //ros::spin();
	 }

  };

  void NEDPlugin::correctW(Vector4d & w2) {
    if(w2(0)<0 && w2(2)<0) {
      w2(0)=0;
      w2(2)=0;
    } else {
      if (w2(0) < 0) {
        w2(2) += w2(0);
        w2(0) = 0;
      }
      if (w2(2) < 0) {
        w2(0) += w2(2);
        w2(2) = 0;
      }
    }

    if(w2(1)<0 && w2(3)<0) {
      w2(1)=0;
      w2(3)=0;
    } else {
      if (w2(1) < 0) {
        w2(3) += w2(1);
        w2(1) = 0;
      }
      if (w2(3) < 0) {
        w2(1) += w2(3);
        w2(3) = 0;
      }
    }

    for (int i=0; i<4; i++)
      if(w2(i)<0)
        w2(i)=0;


  }

  void NEDPlugin::command_cb( geometry_msgs::WrenchConstPtr wrench) {
    //ROS_INFO("Received command!");
    Eigen::Vector4d ftau;
    mav_msgs::Actuators comm;
    comm.angular_velocities.resize(4);
    ftau << wrench->force.z,wrench->torque.x,wrench->torque.y,wrench->torque.z;

    Vector4d w2 = _G.inverse() * ftau;
    correctW(w2);
    comm.header.stamp = ros::Time::now();
    comm.angular_velocities[0] = sqrt(w2(3));
    comm.angular_velocities[1] = sqrt(w2(2));
    comm.angular_velocities[2] = sqrt(w2(1));
    comm.angular_velocities[3] = sqrt(w2(0));
    _commandNED_pub.publish(comm);
  }

  void NEDPlugin::odom_cb( nav_msgs::OdometryConstPtr odom ) {
      tf::Matrix3x3 RNed;
      RNed.setEulerYPR(M_PI/2,0,M_PI);
      tf::Vector3 p;
      tf::Vector3 pDot;
      tf::Vector3 wbb;

      tf::Quaternion q(odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z,  odom->pose.pose.orientation.w);
      tf::Matrix3x3 Rb(q);
      tf::Matrix3x3 RbNed = RNed*Rb*(RNed.transpose());
      tf::Quaternion qNED;
      RbNed.getRotation(qNED);

      p[0] = odom->pose.pose.position.x;
      p[1] = odom->pose.pose.position.y;
      p[2] = odom->pose.pose.position.z;

      p = RNed*p;

      pDot[0] = odom->twist.twist.linear.x;
      pDot[1] = odom->twist.twist.linear.y;
      pDot[2] = odom->twist.twist.linear.z;
      pDot = RNed*pDot;

      wbb[0] = odom->twist.twist.angular.y;
      wbb[1] = odom->twist.twist.angular.x;
      wbb[2] = -odom->twist.twist.angular.z;

      nav_msgs::Odometry odomNED;

      odomNED.header.stamp = ros::Time::now();
      odomNED.header.frame_id = "worldNED";
      odomNED.child_frame_id = "hummingbird/base_linkNED";

      odomNED.pose.pose.position.x = p[0];
      odomNED.pose.pose.position.y = p[1];
      odomNED.pose.pose.position.z = p[2];
      odomNED.pose.pose.orientation.x = qNED.x();
      odomNED.pose.pose.orientation.y = qNED.y();
      odomNED.pose.pose.orientation.z = qNED.z();
      odomNED.pose.pose.orientation.w = qNED.w();

      odomNED.twist.twist.linear.x = pDot[0];
      odomNED.twist.twist.linear.y = pDot[1];
      odomNED.twist.twist.linear.z = pDot[2];
      odomNED.twist.twist.angular.x = wbb[0];
      odomNED.twist.twist.angular.y = wbb[1];
      odomNED.twist.twist.angular.z = wbb[2];

      _odomNED_pub.publish(odomNED);
  }

 // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(NEDPlugin)
}
