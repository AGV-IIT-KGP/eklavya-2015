#ifndef MODE_SWITCHER
#define MODE_SWITCHER

#include <ros/ros.h>
#include <unistd.h>
#include <mutex>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/Float64.h"
#include <iostream>
#include "modeswitcher.h"


 

using namespace std; 
using namespace ros;


class ModeSwitcher {
	
private:
	double d;//distance between steering and the back tires in meters
	
	double maxalpha,minalpha;
	double w_max,w_min;
	
	std::mutex Vx_Xbox_lock ,Vy_Xbox_lock, Vz_Xbox_lock, Vl_Vr_a_lock, W_Xbox_lock , Vx_planner_lock , W_planner_lock , xbox_flag_lock;

	float alpha;

	float W_xbox;
	float Vx_Xbox;
	float Vy_Xbox;
	float Vz_Xbox;

	float W_Planner;
	float Vx_Planner;
	
	
	float Vl_a;
	float Vr_a ;
	
	double  Max_Xbox_Vx;
	//node handler for xbox data
	int xboxflag; //flag to tell whether xbox is currently sending data or not

	geometry_msgs::Twist finaltwist ;
	std_msgs::Float64 finalvt;
	
	
	
public:

	ModeSwitcher();
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	void planCallback(const geometry_msgs::Twist::ConstPtr& pose);
	
	void publish(int argc, char** argv);


};

#endif
