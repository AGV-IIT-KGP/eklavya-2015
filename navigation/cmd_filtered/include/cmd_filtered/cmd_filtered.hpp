#include <iostream>
#include <ros/ros.h>
#include <stdlib.h>
#include <string>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>

#define buffer_size 10
#define loop_rate_hz 10

class CMD_FILTERED
{

public:
	double instructed_omega_;
	double bot_omega_;
	//bool subscription_started_cmd_vel;
	//bool subscription_started_imu;

	geometry_msgs::Twist cmd_vel_;

	CMD_FILTERED();
	void set_instructed_omega(geometry_msgs::Twist cmd_vel);
	void set_bot_omega(sensor_msgs::Imu imu_data);
};