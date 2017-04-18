#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/Imu.h>
#include <boost/assign.hpp>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

#define PI 3.14159265359

sensor_msgs::Imu msg;
ros::Publisher imu_pub;

void imuCallback(const sensor_msgs::Imu msg)
{
  if(ros::ok())
  {
  	sensor_msgs::Imu imu;
  	imu=msg;
  	imu.orientation_covariance=boost::assign::list_of(0.001)(0.0)(0.0)
  		                                                (0.0)(0.001)(0.0)
  		                                                (0.0)(0.0)(0.0001);

    imu.angular_velocity_covariance=boost::assign::list_of(0.001)(0.0)(0.0)
                                                      (0.0)(0.001)(0.0)
                                                      (0.0)(0.0)(0.0001);
    imu.header.frame_id="imu";

    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

   //convert to ENU
   pitch = pitch+PI;

   ROS_INFO_STREAM("NED Yaw: "<<yaw*180/PI<<"\n");

   yaw=yaw+PI/2;

   ROS_INFO_STREAM("ENU Yaw: "<<yaw*180/PI<<"\n");

   pitch=0.0;
   roll=0.0;
   //yaw*=180/PI;

   tf::Quaternion q;
   q.setRPY(tfScalar(roll), tfScalar(pitch), tfScalar(yaw));

   geometry_msgs::Quaternion odom_quat;

   //ROS_INFO_STREAM("tf.x = "<<q.x<<"\n");

   tf::quaternionTFToMsg(q, odom_quat);

   imu.orientation = odom_quat;
   


   //publish
   imu_pub.publish(imu);
  }
}



int main(int argc, char** argv){
  ros::init(argc, argv, "imu_node");

  ros::NodeHandle n;
  ros::Subscriber imu_sub=n.subscribe<sensor_msgs::Imu>("vn_ins/imu",50,imuCallback);
  imu_pub = n.advertise<sensor_msgs::Imu>("/imu", 50);
  ros::spin();
  return 0;
}

