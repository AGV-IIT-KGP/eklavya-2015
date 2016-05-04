#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <fcntl.h>
#include <std_msgs/Float32.h>
#include <controls_msgs/encoder_msg.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <mutex>
#include "BlackPWM.h"
#include <cstdint>

#define PI 3.14159265
//#define Vx_a ( (Vl_a+Vr_a)/2 )/( cos( (Alpha_a * PI)/180)) 



using namespace std; 
using namespace BlackLib;
using namespace ros;

class VxPidB {
	
private:
	
	//ouble Alpha_max;
	//double Alpha_min;
	//double d;         // Front wheel center to rear wheel line center distance
	//double r;		  // Rear wheel center to center of line joining distance	

	double Vr_a , Vl_a;
	double Vx_t;   
	double Alpha_a;
	double Kp_Vx, Ki_Vx, Kd_Vx;
	double Vx_error_sum, Vx_error_diff, Vx_error_old , Vx_error_integral;
	double PWM_Duty_Cycle;
	//int    Vs_PID_loop_rate;

	int PWM_min_percent, PWM_max_percent;
	double PWM_PERIOD_TIME;     // in ns
	
	int vx_pid_loop_rate;
	
	std::mutex Vx_t_lock , Alpha_lock , Vl_Vr_a_lock;

public:

	VxPidB();
	double getMinMax(int Cur_Var,int max,int min);
	void vxTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg);
	//void Alpha_actual_callback(const std_msgs::Float64::ConstPtr& msg);
	void encoderCallback(const controls_msgs::encoder_msg::ConstPtr& msg);
	void implementPid(int argc, char** argv);

};

using namespace std;

VxPidB::VxPidB() :
    Vx_t_lock(), Alpha_lock(), Vl_Vr_a_lock()
{

	Vr_a=0;
	Vl_a=0;
	Vx_t=0;   
	Alpha_a=0;
	Kp_Vx=0;
	Ki_Vx=0;
	Kd_Vx=0;
	Vx_error_sum=0; 
	Vx_error_diff=0;
	Vx_error_old=0;
	Vx_error_integral=0;
	PWM_Duty_Cycle=0;
	vx_pid_loop_rate=0;
	//int    Vs_PID_loop_rate;

	PWM_min_percent=0; PWM_max_percent=0;
	PWM_PERIOD_TIME=0;     // in ns
	
	
}

double VxPidB::getMinMax(int Cur_Var, int max, int min)
{

  if (Cur_Var > max)
    return max;
  else if (Cur_Var < min)
    return min;
  else
    return Cur_Var;
}

void VxPidB::vxTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

  Vx_t_lock.lock();
  Alpha_lock.lock();
  Vx_t = (msg->linear.x);
  //	Vs_t= (msg->linear.x )/( cos( (Alpha_a * PI)/180));
  Alpha_lock.unlock();
  Vx_t_lock.unlock();

}

/*
 void VxPidB::Alpha_actual_callback(const std_msgs::Float64::ConstPtr& msg)
 {  
 ROS_INFO("\n Vs_PID_node: Alpha received \n");
 
 Alpha_lock.lock();
 Alpha_a=msg->data;
 Alpha_lock.unlock();
 
 }
 */



void VxPidB::implementPid(int argc, char** argv)
{

	ros::init(argc, argv, "VxPidB_node");

	ros::NodeHandle nh_;
	ros::Subscriber Override_Subscriber = nh_.subscribe<geometry_msgs::Twist>("target_pose", 5,
																		&VxPidB::vxTargetUpdateCallback, this);
	//ros::Subscriber Alpha_Actual_Subscriber = nh_.subscribe<std_msgs::Float64>("alpha_val_actual" , 5 , Alpha_actual_callback);



	nh_.getParam("/VxPidB_node/Kp_Vx", Kp_Vx);
	nh_.getParam("/VxPidB_node/Ki_Vx", Ki_Vx);
	nh_.getParam("/VxPidB_node/Kd_Vx", Kd_Vx);
	//nh_.getParam("/Vs_PID/Vs_PID_loop_rate", Vs_PID_loop_rate);
	nh_.getParam("/VxPidB_node/PWM_min", PWM_min_percent);
	nh_.getParam("/VxPidB_node/PWM_max", PWM_max_percent);
	nh_.getParam("/VxPidB_node/PWM_PERIOD_TIME", PWM_PERIOD_TIME);
	nh_.getParam("/VxPidB_node/vx_pid_loop_rate", vx_pid_loop_rate);
	
	// nh_.getParam("d", d); 					       // Front wheel center to rear wheel line center distance
	// nh_.getParam("r", r);					       // Rear wheel center to center of line joining distance

	BlackLib::BlackPWM pwm_signal_pin(BlackLib::EHRPWM2A);

	pwm_signal_pin.setPeriodTime(5000000);

	ros::Rate loop_rate(10);

  while (ros::ok())
  {

    /*
     Vl_Vr_a_lock.lock();
     Vx_t_lock.lock();
     Alpha_lock.lock();
     double Vs_error = Vs_t - Vs_a;
     Alpha_lock.unlock();
     Vx_t_lock.unlock();	   	
     Vl_Vr_a_lock.unlock();
     */

  
    Vx_t_lock.lock();
    Alpha_lock.lock();
    double PWM_Duty_Cycle = Vx_t;
    Alpha_lock.unlock();
    Vx_t_lock.unlock();


   
    PWM_Duty_Cycle = getMinMax(PWM_Duty_Cycle, 85, 20);

    pwm_signal_pin.setDutyPercent(PWM_Duty_Cycle);

    ros::spin();

    loop_rate.sleep();

  }
}

int main(int argc, char** argv)
{

  VxPidB * vxPidB = new VxPidB();

  vxPidB->implementPid(argc, argv);

  delete vxPidB;

}

