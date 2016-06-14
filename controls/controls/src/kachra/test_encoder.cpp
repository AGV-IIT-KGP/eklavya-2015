#include "ros/ros.h"
#include "controls_msgs/encoder_msg.h"
#include <iostream>

void chatterCallback(const controls_msgs::encoder_msg::ConstPtr& msg)
{
  std::cout<<"Left Wheel: "<< msg->left_vel << "Right Wheel: "<< msg->right_vel << std::endl; 
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_encoder");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("encoders", 1000, chatterCallback);

  ros::spin();

  return 0;
}
