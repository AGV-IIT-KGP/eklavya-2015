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

int W_PID_loop_rate;


double Get_Min_Max(int Cur_Var,int max,int min){ 
	
	if(Cur_Var>max)
      return max;
    else if(Cur_Var<min)  
      return min;
    else
      return Cur_Var;
  }

void Encoder_Callback(const controls::encoder_msg::ConstPtr& msg)
 {

  ROS_INFO("\n Ws_PID: Encoder Reading Received \n");

	Vl_Vr_a_lock.lock();
		Vl_a=msg->left_vel;
		Vr_a=msg->right_vel ;
	Vl_Vr_a_lock.unlock(); 
 
 }


void Vxt_target_update_callback(const geometry_msgs::Twist::ConstPtr& msg){

	W_t_Lock.lock();
		W_t=msg->angular.z;
	W_t_Lock.unlock();

}
//------------------------------------------------------------


int main (int argc, char** argv) 
{


ros::init(argc, argv,"W_PID");

  ros::NodeHandle pid_nh_; 

  ros::Subscriber Override_Subscriber = pid_nh_.subscribe<geometry_msgs::Twist>("target_pose" , 5 , Vxt_target_update_callback);
  ros::Subscriber Encoder_Subscriber = pid_nh_.subscribe<controls::encoder_msg>("encoders" , 5 , Encoder_Callback);
 
  ros::Publisher alpha_pub = pid_nh_.advertise<std_msgs::Float64>("alpha_val_manipulated", 100); 
  ros::Publisher wa_pub = pid_nh_.advertise<std_msgs::Float64>("wa", 100); 
  ros::Publisher wt_pub = pid_nh_.advertise<std_msgs::Float64("wt", 100); 


pid_nh_.getParam("/W_PID/Kp_W", Kp_W);
pid_nh_.getParam("/W_PID/Ki_W", Ki_W);
pid_nh_.getParam("/W_PID/Kd_W", Kd_W);
pid_nh_.getParam("/W_PID/W_PID_loop_rate", W_PID_loop_rate);


pid_nh_.getParam("Alpha_max", Alpha_max);
pid_nh_.getParam("Alpha_min", Alpha_min);
pid_nh_.getParam("d", d); 					// Front wheel center to rear wheel line center distance
pid_nh_.getParam("r", r);					// Rear wheel center to center of line joining distance
         
	  ros::Rate loop_rate(W_PID_loop_rate);

  std_msgs::Float64 alpha_msg;
  std_msgs::Float64 wa_msg;
  std_msgs::Float64 wt_msg;
  
  while( ros::ok() ){
  
	Vl_Vr_a_lock.lock();
		W_t_Lock.lock();
			double W_error = W_t - W_a;
			wa_msg.data=W_a;
			wt_msg.data=W_t;
		W_t_Lock.unlock();   	
	Vl_Vr_a_lock.unlock();


	W_error_diff = W_error_old - W_error;
	W_error_sum += W_error;
	W_error_old = W_error;

	W_error_integral = Get_Min_Max( (W_error_sum)*Ki_W , Alpha_max , Alpha_min);
	
		W_t_Lock.unlock();   	
	Vl_Vr_a_lock.unlock();


	W_error_diff = W_error_old - W_error;
	W_error_sum += W_error;
	W_error_old = W_error;

	W_error_integral = Get_Min_Max( (W_error_sum)*Ki_W , Alpha_max , Alpha_min);
	
	double Alpha_manipulated = (W_error)*Kp_W + (W_error_integral) + (W_error_diff)*Kd_W;
	
	Alpha_manipulated = Get_Min_Max( Alpha_manipulated , Alpha_max , Alpha_min );

	alpha_msg.data = Alpha_manipulated;

	alpha_pub.publish(alpha_msg);
	

	wa_pub.publish(wa_msg);
	
	wt_pub.publish(wt_msg);
	
	ros::spinOnce();
	loop_rate.sleep();

	}


}
  

