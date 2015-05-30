#ifndef ODOM_AND_IMU
#define ODOM_AND_IMU

#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include "tf/transform_datatypes.h"
#include "encoder_msg.h"
#include <ros/time.h>
//#include "LinearMath/btMatrix3x3.h"
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <geometry_msgs/Twist.h>

class OdometryImuCombiner {

private:
	ros::Time current_time;
    ros::Time last_time;
	sensor_msgs::Imu imu_msg;
	nav_msgs::Odometry odom_msg,odom;
	geometry_msgs::Quaternion quat;
	int count;
	double vl, vr, vx, vy, th, v, x, y, prev_yaw;

public:
	OdometryImuCombiner();
	void odomCallback(const geometry_msgs::Twist msg);
	void imuCallback(sensor_msgs::Imu imu_msg);
	void publishUsing(ros::Publisher & publisher);
};

#endif