#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <boost/assign.hpp>

sensor_msgs::Imu msg;
ros::Publisher imu_pub;

void imuCallback(const sensor_msgs::Imu msg)
{
  if(ros::ok())
  {
  	sensor_msgs::Imu imu;
  	imu=msg;
  	imu.orientation_covariance=boost::assign::list_of(1.0e+9)(0.0)(0.0)
  		                                                (0.0)(1.0e+9)(0.0)
  		                                                (0.0)(0.0)(0.09);

    imu.angular_velocity_covariance=boost::assign::list_of(1.0e+9)(0.0)(0.0)
                                                      (0.0)(1.0e+9)(0.0)
                                                      (0.0)(0.0)(0.001);


   imu_pub.publish(imu);
  }
}



int main(int argc, char** argv){
  ros::init(argc, argv, "imu_node");

  ros::NodeHandle n;
  ros::Subscriber imu_sub=n.subscribe<sensor_msgs::Imu>("vn_ins/imu",50,imuCallback);
  imu_pub = n.advertise<sensor_msgs::Imu>("imu", 50);
  ros::spin();
  return 0;
}

