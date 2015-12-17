#include <ros/ros.h>
#include <boost/assign.hpp>
#include <sensor_msgs/NavSatFix.h>
#include "ros/time.h"
sensor_msgs::NavSatFix msg;
sensor_msgs::NavSatFix origin,temp;
ros::Publisher gps_pub;
ros::Publisher origin_pub;
volatile int i=0;
  //ros::Time current_time;
  //ros::Time last_time;

  
void odomCallback(sensor_msgs::NavSatFix msg)
{
	if (ros::ok())
{
    if(i<100)
    {
      i++;
      temp.latitude+=msg.latitude;
      temp.longitude+=msg.longitude;
    }
    if(i==100)
    {
      i++;
      origin.latitude=temp.latitude/100;
      origin.longitude=temp.longitude/100;
     }
    //add covariance
    msg.position_covariance_type=2;
    msg.position_covariance=boost::assign::list_of(1.0)(0.0)(0.0)
                                                  (0.0)(1.0)(0.0)
                                                  (0.0)(0.0)(1.0);
                                               

        //publish the message

    msg.header.frame_id="gps";
                                                  
    gps_pub.publish(msg);
    origin_pub.publish(origin);
    //last_time = current_time;

    
  }
}


int main(int argc, char** argv){
  ros::init(argc, argv, "gps");

  ros::NodeHandle n;
  //gps covariance
  ros::Subscriber gps_sub=n.subscribe<sensor_msgs::NavSatFix>("vn_ins/fix",50,odomCallback);
  gps_pub = n.advertise<sensor_msgs::NavSatFix>("fix_c", 50);
  origin_pub=n.advertise<sensor_msgs::NavSatFix>("gps/origin",50);
  ros::spin();
  return 0;
}