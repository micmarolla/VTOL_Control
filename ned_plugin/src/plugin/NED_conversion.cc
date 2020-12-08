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

using namespace Eigen;
using namespace std;

namespace gazebo
{
  class NEDPlugin : public WorldPlugin
  {

	private: ros::NodeHandle* _nh;
  private:
     ros::Publisher _odomNED_pub, _commandNED_pub;
     ros::Subscriber _odom_sub, _command_sub;
     Matrix4d _G;
  public:
   void odom_cb( nav_msgs::OdometryConstPtr );
   void command_cb( geometry_msgs::WrenchConstPtr );
   void correctW(Vector4d & w2);
	 void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {
		_nh = new ros::NodeHandle();

    ROS_INFO("NED conversions plugin started!");
    _odom_sub = _nh->subscribe("/hummingbird/ground_truth/odometry", 0, &NEDPlugin::odom_cb, this);
    _odomNED_pub = _nh->advertise< nav_msgs::Odometry>("/hummingbird/ground_truth/odometryNED", 0);
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

    for (int i=0; i<4; i++) {
      if(w2(i)<0) w2(i)=0;
      //ROS_INFO("w(%d): %f",i,w2(i));
    }

  }

  void NEDPlugin::command_cb( geometry_msgs::WrenchConstPtr wrench) {
    //ROS_INFO("Received command!");
    Eigen::Vector4d ftau;
    mav_msgs::Actuators comm;
    comm.angular_velocities.resize(4);
    ftau << wrench->force.z,wrench->torque.x,wrench->torque.y,wrench->torque.z;

    Vector4d w2 = _G.inverse() * ftau;
    //ROS_INFO("G inverted!");
    correctW(w2);
    //ROS_INFO("W corrected!");
    comm.header.stamp = ros::Time::now();
    comm.angular_velocities[0] = sqrt(w2(3));
    comm.angular_velocities[1] = sqrt(w2(2));
    comm.angular_velocities[2] = sqrt(w2(1));
    comm.angular_velocities[3] = sqrt(w2(0));
    //ROS_INFO("sqrt computed!");
    _commandNED_pub.publish(comm);
    //ROS_INFO("Command published!");
  }

  void NEDPlugin::odom_cb( nav_msgs::OdometryConstPtr odom ) {
      tf::Matrix3x3 RNed;
      RNed.setEulerYPR(M_PI/2,0,M_PI);
      //RNed = RNed.transpose();
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
      //cout<<odom->twist.twist.linear.x;
      pDot = RNed*pDot;
      //cout<<pDot[0];
      //pDot = RNed*Rb*pDot*RNed.transpose();

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
      //ROS_INFO("x: %f  y: %f z: %f",_P_dot(0),_P_dot(1),_P_dot(2));
      //ROS_INFO("x: %f  y: %f z: %f",_P(0),_P(1),_P(2));
      //ROS_INFO("phi: %f  tetha: %f psi: %f",_Eta(0)*180.0/M_PI,_Eta(1)*180.0/M_PI,_Eta(2)*180.0/M_PI);
  }

 // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(NEDPlugin)
}
