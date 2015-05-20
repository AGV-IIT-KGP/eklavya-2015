#include <yaw_publisher.hpp> 


void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  ros::NodeHandle n;
  tf::quaternionMsgToTF(msg->orientation, quat);
  tf::Matrix3x3(quat).getRPY(r,p,y);

   ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("/vn_ins/yaw", 1000);
  
  std_msgs::Float64 msg1;
  msg1.data=(float)y;
  ROS_INFO("\n%f",msg1.data);
  chatter_pub.publish(msg1);

  ros::spinOnce();
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "dummy_publisher");
  ros::NodeHandle n1;
 
  
  ros::Subscriber sub = n1.subscribe("/imu", 1000, imuCallback);

  ros::spin();
 
  return 0;
}