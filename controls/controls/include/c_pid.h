#ifndef C_PID
#define C_PID

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <fcntl.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <mutex>



class CPid {

private:
	
	std::mutex C_t_Lock, C_a_Lock , Alpha_Lock , Vl_Vr_a_lock;

	double W_t,W_a;
	float curve_t, curve_a;
	double Vr_a , Vl_a;
	double Vx_a;
	
	double Alpha_a, Alpha_t;

	double Kp_C, Ki_C, Kd_C;

	double Alpha_max;
	double Alpha_min;
	double d;         // Front wheel center to rear wheel line center distance
	double r;		  // Rear wheel center to center of line joining distance	

	double c_error_integral, c_error_diff, c_error_old;
	int c_pid_loop_rate;
	
	double count_max;
	double count_min;
	double vDead; 
	
	
public:

	CPid();
	double getMinMax(int Cur_Var,int max,int min);
	void encoderCallback(const geometry_msgs::Twist::ConstPtr &);
	void wTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void implementPid(int argc, char** argv);
	void vninsCallback(const geometry_msgs::Twist::ConstPtr &msg);

	};
	
#endif
