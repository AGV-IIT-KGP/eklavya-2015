#ifndef VX_PID
#define VX_PID

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <fcntl.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <mutex>
#include <cstdint>

#define PI 3.14159265
//#define Vx_a ( (Vl_a+Vr_a)/2 )/( cos( (Alpha_a * PI)/180)) 

#define Vx_a ( (Vl_a+Vr_a)/2 ) 

using namespace std; 
using namespace ros;

class VxPid {
	
private:
	
	//ouble Alpha_max;
	//double Alpha_min;
	//double d;         // Front wheel center to rear wheel line center distance
	//double r;		  // Rear wheel center to center of line joining distance	

	double Vr_a , Vl_a;
	double Vx_t,Vy_t,Vz_t;   
	double Alpha_a;
	double Kp_Vx, Ki_Vx, Kd_Vx;
	double Vx_error_diff, Vx_error_old , Vx_error_integral;
	double PWM_Duty_Cycle;
	//int    Vs_PID_loop_rate;

	int PWM_min_percent, PWM_max_percent;
	double PWM_PERIOD_TIME;     // in ns
	
	int vx_pid_loop_rate;
	
	std::mutex Vx_t_lock , Alpha_lock , Vl_Vr_a_lock, Vy_t_lock, manualmode_lock;

public:

	VxPid();
	double getMinMax(int Cur_Var,int max,int min);
	void vxTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg);
	//void Alpha_actual_callback(const std_msgs::Float64::ConstPtr& msg);
	void encoderCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void implementPid(int argc, char** argv);

};




#endif
