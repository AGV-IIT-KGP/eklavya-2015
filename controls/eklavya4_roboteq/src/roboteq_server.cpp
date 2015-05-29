#include <iostream>
#include <stdio.h>
#include <string.h>

#include "ros/ros.h"
#include "eklavya4_roboteq/SetSpeed.h" 

#include "eklavya4_roboteq/RoboteqDevice.h"
#include "eklavya4_roboteq/ErrorCodes.h"
#include "eklavya4_roboteq/Constants.h"

using namespace std;

RoboteqDevice device;
int status = 0;

bool setSpeed(eklavya4_roboteq::SetSpeed::Request &req, eklavya4_roboteq::SetSpeed::Response &res) {
    
    res.code = 0;
    
	if((status = device.SetCommand(_GO, 1, req.stearing_angle)) != RQ_SUCCESS) {
		ROS_INFO("Failed... Error code --> %d", status);
	}
	else {
		ROS_DEBUG("Succeeded.");
	}
	
	usleep(100);
	
	int numberOfattempts = 0;
	
	while (status != 0 && numberOfattempts < 10) {
		
		usleep(500000);
		ROS_INFO("Attempting server restart...");
		device.Disconnect();
		status = device.Connect("/dev/serial/by-id/usb-Roboteq_Motor_Controller_498954A73235-if00");
		if (status == 0) {
			ROS_INFO("Connection re-established...");
		}
		numberOfattempts++;
		
	}
	
	if (numberOfattempts == 10 && status != 0) {
		ROS_ERROR("Could not connect to Roboteq Motor Controller... Aborting operation...");
		res.code = -1;
	}
	
    ROS_INFO("sending back response: [%ld]", (long int)res.code);
    
    return true;
}

/*bool getSpeed(eklavya_roboteq::GetSpeed::Request &req, eklavya_roboteq::GetSpeed::Response &res) {
	ROS_INFO("request: Encoder speed data.");
	
	int left_speed = 0, right_speed = 0;
    
	if((status = device.GetValue(_ABSPEED, 1, left_speed)) != RQ_SUCCESS) {
		ROS_INFO("Failed... Error code --> ", status);
	}
	else {
		ROS_DEBUG("Succeeded.");
	}
		
	usleep(100);
	
	if (status == 0) {
		
		if((status = device.GetValue(_ABSPEED, 2, right_speed)) != RQ_SUCCESS) {
			ROS_INFO("Failed... Error code --> ", status);
		}
		else {
			ROS_DEBUG("Succeeded.");
			return true;
		}
	}
		
	res.left_speed = left_speed;
	res.right_speed = right_speed;
	
	return true;
}*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "motor_controller_server");
    ros::NodeHandle n;
    
    string response = "";
    
    ROS_INFO("\n\n--- Roboteq Motor Controller Request Gateway Server ---\n");
    ROS_INFO("Initializing...");
	usleep(500000);

	status = device.Connect("/dev/serial/by-id/usb-Roboteq_Motor_Controller_498954A73235-if00");
	//status = device.Connect("/dev/ttyACM0");

	while (status != RQ_SUCCESS && ros::ok())
	{
		ROS_INFO("Error connecting to device: %d", status);
		ROS_INFO("Attempting server restart...");
		usleep(999999);
		device.Disconnect();
		status = device.Connect("/dev/serial/by-id/usb-Roboteq_Motor_Controller_498954A73235-if00");
		if (status == 0) {
			ROS_INFO("Connection re-established...");
		}
	}
  
    ros::ServiceServer service1 = n.advertiseService("motor_controller", setSpeed);
    //ros::ServiceServer service2 = n.advertiseService("motor_speed", getSpeed);
    ROS_INFO("Server initialized...");
    ROS_INFO("Ready to control motors...");
    
    ros::spin();
	
	device.Disconnect();
  
    return 0;
}
