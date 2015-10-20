#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <fcntl.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <controls_msgs/encoder_msg.h>
#include <math.h>

void encoderCallback(const controls_msgs::encoder_msg::ConstPtr& msg);
void wTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg);

ros::Subscriber Override_Subscriber = nh_.subscribe<geometry_msgs::Twist>("target_pose", 5, &VxPid::vxTargetUpdateCallback, this);
ros::Subscriber Override_Subscriber = pid_nh_.subscribe<geometry_msgs::Twist>("target_pose", 5, &WPid::wTargetUpdateCallback, this);
ros::Rate loop_rate(10);

void WPid::wTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  W_t = (msg->angular.z);
}
void VxPid::vxTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
  Vx_t = (msg->linear.x);
}

int main(int argc,char **argv)
{
