#ifndef OVERRIDING_LAYER
#define OVERRIDING_LAYER

#include <ros/ros.h>
#include <unistd.h>
#include <mutex>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Twist.h>
#include "BlackGPIO.h"
#include <iostream>
#include "overriding_layer.h"

#define RAD_FACTOR 318.47

using namespace std; 
using namespace BlackLib;
using namespace ros;


class OverridingLayer {
	
private:
	double d;//distance between steering and the back tires in meters
	float MULTI_FACTOR;//to convert -32767 to 32767 to -500 to 500
	
	double maxalpha,minalpha;

	std::mutex Vx_Xbox_lock , W_Xbox_lock , Vx_planner_lock , W_planner_lock , xbox_flag_lock , planner_flag_lock, estoplock;

	float alpha;

	float W_xbox;
	float Vx_Xbox;
	double Max_Xbox_Vx;


	float W_Planner;
	float Vx_Planner;

	//node handler for xbox data
	int estopflag; //flag to tell whether estop is currently enabled
	int xboxflag; //flag to tell whether xbox is currently sending data or not
	int planflag;

	float xboxthreshold;
	geometry_msgs::Twist finaltwist ;
	
public:

	OverridingLayer();
	
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
	
	void planCallback(const geometry_msgs::Twist::ConstPtr& pose);
	
	void publish(int argc, char** argv);
};

#endif
