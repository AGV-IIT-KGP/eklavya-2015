#include <cmd_filtered/cmd_filtered.hpp>


CMD_FILTERED::CMD_FILTERED()
{
	//subscription_started_cmd_vel = false;
	//subscription_started_imu  = false;
}

void CMD_FILTERED::set_instructed_omega(geometry_msgs::Twist cmd_vel)
{
	cmd_vel_ = cmd_vel;
	instructed_omega_ = cmd_vel.angular.z;
	//subscription_started_cmd_vel = true;
}

void CMD_FILTERED::set_bot_omega(sensor_msgs::Imu imu_data)
{
	bot_omega_ = imu_data.angular_velocity.z; 
	//subscription_started_imu = false;
}