#include <ros/ros.h>
#include <boost/assign.hpp>
#include <nav_msgs/Odometry.h>
#include "ros/time.h"
nav_msgs::Odometry msg;
ros::Publisher odom_pub;
  //ros::Time current_time;
  //ros::Time last_time;

  
void odomCallback(nav_msgs::Odometry msg)
{
	if (ros::ok())
{

    //add covariance
    msg.pose.covariance=boost::assign::list_of(10000.0)(2500.0)(0.0)(0.0)(0.0)(0.0)
                                               (2500.0)(10000.0)(0.0)(0.0)(0.0)(0.0)
                                               (0.0)(0.0)(1.0e+9)(0.0)(0.0)(0.0)
                                               (0.0)(0.0)(0.0)(1.0e+9)(0.0)(0.0)
                                               (0.0)(0.0)(0.0)(0.0)(1.0e+9)(0.0)
                                               (0.0)(0.0)(0.0)(0.0)(0.0)(1.0e+9);

    msg.twist.covariance=boost::assign::list_of(1.0e+9)(0.0)(0.0)(0.0)(0.0)(0.0)
                                               (0.0)(1.0e+9)(0.0)(0.0)(0.0)(0.0)
                                               (0.0)(0.0)(1.0e+9)(0.0)(0.0)(0.0)
                                               (0.0)(0.0)(0.0)(1.0e+9)(0.0)(0.0)
                                               (0.0)(0.0)(0.0)(0.0)(1.0e+9)(0.0)
                                               (0.0)(0.0)(0.0)(0.0)(0.0)(1.0e+9);
    /*for(int i=0;i<5;i++)
    {
      for(int j=0;j<5;j++)
      {
        if(i=j)
        {
          switch(i){
            case 0: msg.pose.covariance[i,j]=0.0625;
                    break;
            case 1: msg.pose.covariance[i,j]=0.09;
                    break;
            case 5: msg.pose.covariance[i,j]=0.1;
                    break;
            default: msg.pose.covariance[i,j]=1.0e+9;
                     break;
          }
          else
            msg.pose.covariance[i,j]=0.0;
        }
      }
    }

    for(int i=0;i<5;i++)
    {
      for(int j=0;j<5;j++)
      {
        if(i=j)
        {
          switch(i){
            case 0: msg.twist.covariance[i,j]=0.0625;
                    break;
            case 1: msg.twist.covariance[i,j]=0.09;
                    break;
            case 5: msg.twist.covariance[i,j]=0.1;
                    break;
            default: msg.twist.covariance[i,j]=1.0e+9;
                     break;
          }
          else
            msg.pose.covariance[i,j]=0.0;
        }
      }
    }*/
   
    msg.header.frame_id="odom_g";
    //publish the message
    odom_pub.publish(msg);
    //last_time = current_time;

    
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odom_gps");

  ros::NodeHandle n;
  //odometry - covariance + tf
  ros::Subscriber odom_sub=n.subscribe<nav_msgs::Odometry>("/odometry/gps",50,odomCallback);
  odom_pub = n.advertise<nav_msgs::Odometry>("odom_g", 50);
  ros::spin();
  return 0;
}