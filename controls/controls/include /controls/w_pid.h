#ifndef VX_PID
#define VX_PID

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <fcntl.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <controls/encoder_msg.h>
#include <math.h>
#include <mutex>

#define W_a  ( (Vr_a-Vl_a)/(2*r) )

class WPid {

private:
	
	std::mutex W_t_Lock , Alpha_Lock , Vl_Vr_a_lock;

	double W_t=0;
	double Vr_a=0 , Vl_a=0;
	double Alpha_a=0 , Alpha_t=0;

	double Kp_W=0 , Ki_W=0 , Kd_W=0;

	double Alpha_max;
	double Alpha_min;
	double d;         // Front wheel center to rear wheel line center distance
	double r;		  // Rear wheel center to center of line joining distance	

	double W_error_sum=0 , W_error_integral=0 , W_error_diff=0 , W_error_old=0;

public:

	WPid();
	double getMinMax(int Cur_Var,int max,int min);
	void encoderCallback(const controls::encoder_msg::ConstPtr& msg);
	void vxTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void implementPid(int argc, char** argv);

	};
	
#endif
