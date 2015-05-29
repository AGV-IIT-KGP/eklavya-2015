#include <iostream>
#include "overriding_layer.h"


using namespace ros;
using namespace BlackLib;


OverridingLayer::OverridingLayer() :
	finaltwist(),estoplock(), Vx_Xbox_lock() , W_Xbox_lock() , Vx_planner_lock() , W_planner_lock() , xbox_flag_lock() , planner_flag_lock()
{
	alpha=0;
	estopflag=0; //flag to tell whether estop is currently enabled
	xboxflag=0; //flag to tell whether xbox is currently sending data or not
	planflag=0;
	MULTI_FACTOR=90/(0.756*2);
	
	
}

void OverridingLayer::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) //main callback function for xbox data
{

  	if ( (joy->axes[5]<0.9)&&(joy->axes[5]>= -0.9) ){
	
	//cout<<"Axes Data in use";
	//cout<<endl;
	xbox_flag_lock.lock();
	xboxflag=1;
	xbox_flag_lock.unlock();
	
	float val = joy->axes[5];       //Reading from the right top trigger
	float duty=100-((val+1)*50.0);  //duty cycle to be sent to the BLDC control function
	
	Vx_Xbox_lock.lock();
	
	Vx_Xbox=(duty/100)*2;
	
	Vx_Xbox_lock.unlock();
	
	}
 
	float dataaxes=joy->axes[2];
	
	if(dataaxes>=0.244) 
	{
		xbox_flag_lock.lock();
		xboxflag=1;
		xbox_flag_lock.unlock();
		
		alpha=(dataaxes-0.244)*MULTI_FACTOR; 
		
		W_Xbox_lock.lock();
		W_xbox=(Vx_Xbox*tan(alpha))/d;
		W_Xbox_lock.unlock();
		
		
	}
	else if(dataaxes<=-0.244)
	{
		xbox_flag_lock.lock();
		xboxflag=1;
		xbox_flag_lock.unlock();
		
		alpha=(dataaxes+0.244)*MULTI_FACTOR;
		
		W_Xbox_lock.lock();
		W_xbox=(Vx_Xbox*tan(alpha))/d;
		W_Xbox_lock.unlock();
		
		
	}
	

////////////////////////////////////////////////////////////////E STOP/////////////////////////////////////////////////////////////////////////////////////////////////


	BlackGPIO E_stop_enable(GPIO_60 ,output);
	
	
	if((joy->buttons[0]==1)) //Setting E-stop on if it is not already on
	{
		E_stop_enable.setValue(high);

		cout<<"E-stop Enabled";

		cout<<endl;
	}

	if((joy->buttons[1]==1)) //Setting E-stop off if it is not already off
	{
		cout<<"E-stop Disabled";
		cout<<endl;
		E_stop_enable.setValue(low);
		
		
	}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

void OverridingLayer::planCallback(const geometry_msgs::Twist::ConstPtr& pose)
{
		cout<<"Planner Data in Use";
		cout<<endl;
		xbox_flag_lock.lock();
		
		if(xboxflag==1)
		{
			return ;
		}
		else if(xboxflag==0)
		{
		planner_flag_lock.lock();
		planflag=1;
		planner_flag_lock.unlock();
		W_Planner = pose->angular.z; //calculate angle required to be sent
		Vx_Planner=pose->linear.x; //add data in transfer variable
		return ;
		}
		xbox_flag_lock.unlock();
}

 void OverridingLayer::publish(int argc, char** argv){
	ros::init(argc, argv, "overriding_layer");
	
	ros::NodeHandle nh_;
	ros::Rate loop_rate(50);
	nh_.getParam("/overriding_layer/maxvelocity", Max_Xbox_Vx);
	nh_.getParam("d",d);
	nh_.getParam("Alpha_Max",maxalpha);
	nh_.getParam("Alpha_Min",minalpha);


	ros::Subscriber joy_sub;
	ros::Subscriber plan_sub;  

  	joy_sub = nh_.subscribe <sensor_msgs::Joy> ("joy", 1000 , &OverridingLayer::joyCallback , this); 
  	plan_sub = nh_.subscribe <geometry_msgs::Twist> ("cmd_vel", 1000 , &OverridingLayer::planCallback, this); 
	
	ros::Publisher send_twist = nh_.advertise<geometry_msgs::Twist>("target_pose", 5);

	while(ros::ok)
	{
		xbox_flag_lock.lock();
		
		planner_flag_lock.lock();
		
		if(xboxflag==1)
		{
			Vx_Xbox_lock.lock();
				finaltwist.linear.x=Vx_Xbox;
			Vx_Xbox_lock.unlock();
			
			W_Xbox_lock.lock();
				finaltwist.angular.z= W_xbox;
			W_Xbox_lock.unlock();
			
			send_twist.publish(finaltwist);
			
			
		}
		else if(xboxflag==0)
		{
			if(planflag==1)
			{	Vx_planner_lock.lock();
					finaltwist.linear.x=Vx_Planner;
				Vx_planner_lock.unlock();
				
				W_planner_lock.lock();
					finaltwist.linear.x=W_Planner;
				W_planner_lock.unlock();
				
				send_twist.publish(finaltwist);

			}
		}
		
		xboxflag=0;
		planflag=0;
			
		planner_flag_lock.unlock();
		xbox_flag_lock.unlock();
		
		ros::spinOnce();
		loop_rate.sleep();
	}
 }
 
 
 
int main(int argc, char** argv)
{	
	
	OverridingLayer * layer = new OverridingLayer();
	
	layer->publish(argc, argv);
	
	delete layer;
}
