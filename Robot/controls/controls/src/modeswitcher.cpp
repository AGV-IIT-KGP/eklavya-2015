
#include <iostream>
#include "modeswitcher.h"
#include <cmath>

using namespace ros;
using namespace std;

ModeSwitcher::ModeSwitcher() :
	Vl_Vr_a_lock() , finaltwist(), Vx_Xbox_lock() , W_Xbox_lock() , Vx_planner_lock() , W_planner_lock() , xbox_flag_lock() 
{

	d=0.7;//distance between steering and the back tires in meters
	
	maxalpha=45 ; minalpha = -45;

	alpha=0;

	W_xbox=0;
	Vx_Xbox=0;

	W_Planner=0;
	Vx_Planner=0;
	
	xboxflag=0; //flag to tell whether xbox is currently sending data or not
	
	Max_Xbox_Vx=2.0;

	w_max=0;
	w_min=0;	
	Vl_a = 0;
	Vr_a = 0;
	
}

void ModeSwitcher::joyCallback(const sensor_msgs::Joy::ConstPtr& joy) //main callback function for xbox data
{

//	cout<< "??????????????" << endl;

	int manual_button=joy->buttons[4];
	int auto_button=joy->buttons[5];
	


	if(auto_button==1 && manual_button==0 )
	{

//		std::cout<< "***************" <<std::endl;
		xbox_flag_lock.lock();
		xboxflag=0;
		xbox_flag_lock.unlock();
		
		return;
	}

	else if(manual_button==1 && auto_button==0)
	{
		xbox_flag_lock.lock();
//		std::cout<< "---------------" <<std::endl;
		
		xboxflag=1;
		xbox_flag_lock.unlock();

	
	}
				xbox_flag_lock.lock();
                int temp_flag1=xboxflag;
                xbox_flag_lock.unlock();

	
	if (temp_flag1==1){

 		float val = joy->axes[5];       //Reading from the right top trigger

		float rescaled_val =1-((val + 1.0 )/2); 

		Vx_Xbox_lock.lock();
		
		Vx_Xbox=( rescaled_val * Max_Xbox_Vx);

		Vx_Xbox_lock.unlock();
	
 
		float alpha_axes=joy->axes[2];

		W_xbox= w_max + ((alpha_axes-1)/2)*( abs(w_min)+abs(w_max));

/*
		alpha= maxalpha + ((alpha_axes-1)/2)*( abs(minalpha)+abs(maxalpha)); 

		
		W_Xbox_lock.lock();
		Vl_Vr_a_lock.lock();
		W_xbox=(Vx_a*tan(alpha))/d;
		Vl_Vr_a_lock.unlock();
		W_Xbox_lock.unlock();
*/	
	}

	return ;
}


void ModeSwitcher::encoderCallback(const controls_msgs::encoder_msg::ConstPtr& msg)
{
  Vl_Vr_a_lock.lock();
  Vl_a = msg->left_vel;
  Vr_a = msg->right_vel;
  Vl_Vr_a_lock.unlock();

}


void ModeSwitcher::planCallback(const geometry_msgs::Twist::ConstPtr& pose)
{		
		xbox_flag_lock.lock();
		int temp_flag=xboxflag;
		xbox_flag_lock.unlock();

		if(temp_flag==1)
		{
			return ;
		}
		else if(temp_flag==0)
		{
			Vx_planner_lock.lock();
			Vx_Planner=pose->linear.x;
			Vx_planner_lock.unlock();
			
			W_planner_lock.lock();
			W_Planner = pose->angular.z;
			W_planner_lock.unlock();
		 //calculate angle required to be sent
		 //add data in transfer variable
		
		}
		return;
}

 void ModeSwitcher::publish(int argc, char** argv){

	ros::init(argc, argv, "modeswitching_node");
	ros::NodeHandle nh_;
	ros::Rate loop_rate(10);
	
	ros::Subscriber joy_sub;
	ros::Subscriber plan_sub;  

  	joy_sub = nh_.subscribe <sensor_msgs::Joy> ("joy", 1000 , &ModeSwitcher::joyCallback , this); 
  	plan_sub = nh_.subscribe <geometry_msgs::Twist> ("cmd_vel", 1000 , &ModeSwitcher::planCallback, this); 
  	
	nh_.getParam("/modeswitching_node/maxvelocity", Max_Xbox_Vx);
	nh_.getParam("d",d);
	nh_.getParam("Alpha_Max",maxalpha);
	nh_.getParam("Alpha_Min",minalpha);
	
	nh_.getParam("w_max",w_max);
	nh_.getParam("w_min",w_min);
	

	ros::Publisher send_twist = nh_.advertise<geometry_msgs::Twist>("target_pose", 5);

	while(ros::ok)
	{	
			
		xbox_flag_lock.lock();
        int temp_flag=xboxflag;
		xbox_flag_lock.unlock();

		if(temp_flag==1)
		{
			cout<<"XXX"; //  << "Vx from xbox: "<< Vx_Xbox <<" Wx from xbox: "<< W_xbox<<std::endl;
			cout<<endl;
				
				Vx_Xbox_lock.lock();
			 
				finaltwist.linear.x=Vx_Xbox;
			//	cout;
				cout<<endl;
				Vx_Xbox_lock.unlock();
				
				W_Xbox_lock.lock();
				finaltwist.angular.z= W_xbox;
			//	cout;
				cout<<endl;
				W_Xbox_lock.unlock();
				
				send_twist.publish(finaltwist);
		}
		
		else if(temp_flag==0)
		{
			cout<<"PPP"; //<<" Vx from planner: "<< Vx_Planner <<" W from planner: "<<W_Planner <<std::endl;
			
				Vx_planner_lock.lock();
					finaltwist.linear.x=Vx_Planner;
			//		cout;
                                cout<<endl;

				Vx_planner_lock.unlock();

				W_planner_lock.lock();
					finaltwist.linear.x=W_Planner;
					finaltwist.linear.x=Vx_Planner;
                //    cout;
					cout<<endl;

				W_planner_lock.unlock();

		}

		ros::spinOnce();
		loop_rate.sleep();
	}
 }
 
 
 
int main(int argc, char** argv)
{	

	ModeSwitcher * layer = new ModeSwitcher();
	
	layer->publish(argc, argv);
	
	delete layer;
}
