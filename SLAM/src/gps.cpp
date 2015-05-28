#include <ros/ros.h>
#include <boost/assign.hpp>
#include <sensor_msgs/NavSatFix.h>
#include "ros/time.h"
sensor_msgs::NavSatFix msg;
ros::Publisher gps_pub;
  //ros::Time current_time;
  //ros::Time last_time;

  
void odomCallback(sensor_msgs::NavSatFix msg)
{
	if (ros::ok())
{

    //add covariance
    msg.position_covariance_type=2;
    msg.position_covariance=boost::assign::list_of(5.0)(0.0)(0.0)
                                                  (0.0)(5.0)(0.0)
                                                  (0.0)(0.0)(1.0e+9);
                                               

        //publish the message
    gps_pub.publish(msg);
    //last_time = current_time;

    
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "gps");

  ros::NodeHandle n;
  //gps covariance
  ros::Subscriber gps_sub=n.subscribe<sensor_msgs::NavSatFix>("vn_ins/fix",50,odomCallback);
  gps_pub = n.advertise<sensor_msgs::NavSatFix>("fix_c", 50);
  ros::spin();
  return 0;
}