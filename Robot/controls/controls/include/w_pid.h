#ifndef W_PID
#define W_PID

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <fcntl.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <mutex>



class WPid {

private:
	
	std::mutex W_t_Lock , Alpha_Lock , Vl_Vr_a_lock;

	double W_t,W_a;
	double Vr_a , Vl_a;
	double Alpha_a, Alpha_t;

	double Kp_W, Ki_W, Kd_W;

	double Alpha_max;
	double Alpha_min;
	double d;         // Front wheel center to rear wheel line center distance
	double r;		  // Rear wheel center to center of line joining distance	

	double W_error_sum, W_error_integral, W_error_diff, W_error_old;
	int w_pid_loop_rate;
	
	double count_max;
	double count_min;
	
public:

	WPid();
	double getMinMax(int Cur_Var,int max,int min);
	void encoderCallback(const geometry_msgs::Twist::ConstPtr &);
	void wTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void implementPid(int argc, char** argv);

	};
	
#endif
