#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include "ros/time.h"
geometry_msgs::Pose2D msg;
ros::Publisher odom_pub;
const double l=0.67;         //length of the axle
//const double c=0.0212790542; //(2*pi*r)/60, Radius(r) = 8 inch


double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  double v;
  double vl,vr;

  ros::Time current_time;
  ros::Time last_time;

  
void odomCallback(const geometry_msgs::Pose2D msg)
{
	if (ros::ok())
{
    vl=msg.x;
    vr=msg.y;
    current_time = ros::Time::now();

    double dt = (current_time - last_time).toSec();
    v=(vl+vr)/2;
    vth=((vr-vl)/l);
    double delta_th = vth * dt;
    vx=v*cos(delta_th);
    vy=v*sin(delta_th);



    //compute odometry in a typical way given the velocities of the robot
    
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    

    x += delta_x;
    y += delta_y;
    th += delta_th;
    tf::TransformBroadcaster odom_broadcaster;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);

    last_time = current_time;

    
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle n;
  last_time=ros::Time::now();
  ros::Subscriber odom_sub=n.subscribe<geometry_msgs::Pose2D>("encoders",50,odomCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::spin();
  return 0;
}

  

 