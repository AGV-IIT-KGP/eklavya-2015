#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <stdio.h>
#include <fcntl.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <controls_msgs/encoder_msg.h>
#include <math.h>
#include <mutex>

#define W_a  ( (Vr_a-Vl_a)/(2*r) )

class WPidB {

private:
	
	std::mutex W_t_Lock , Alpha_Lock , Vl_Vr_a_lock;

	double W_t;
	double Vr_a , Vl_a;
	double Alpha_a, Alpha_t;

	double Kp_W, Ki_W, Kd_W;

	double Alpha_max;
	double Alpha_min;
	double d;         // Front wheel center to rear wheel line center distance
	double r;		  // Rear wheel center to center of line joining distance	

	double W_error_sum, W_error_integral, W_error_diff, W_error_old;
	int w_pid_loop_rate;

public:

	WPidB();
	double getMinMax(int Cur_Var,int max,int min);
	void wTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg);
	void implementPid(int argc, char** argv);

	};
	



WPidB::WPidB() :
    W_t_Lock(), Alpha_Lock(), Vl_Vr_a_lock()
{

	W_t = 0;
	Vr_a = 0;
	Vl_a = 0;
	Alpha_a = 0;
	Alpha_t = 0;
	Kp_W = 0;
	Ki_W = 0;
	Kd_W = 0;
	Alpha_max=0;
	Alpha_min=0;
	r=0; d=0;
	W_error_sum=0; W_error_integral=0; W_error_diff=0; W_error_old=0;
	w_pid_loop_rate=10;

}

double WPidB::getMinMax(int Cur_Var, int max, int min)
{

  if (Cur_Var > max)
    return max;
  else if (Cur_Var < min)
    return min;
  else
    return Cur_Var;
}


void WPidB::wTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

  W_t_Lock.lock();
  W_t = msg->angular.z;
  W_t_Lock.unlock();

}

void WPidB::implementPid(int argc, char** argv)
{

  ros::init(argc, argv, "WPidB_node");

  ros::NodeHandle pid_nh_;

  ros::Subscriber Override_Subscriber = pid_nh_.subscribe<geometry_msgs::Twist>("target_pose", 5,
                                                                                &WPidB::wTargetUpdateCallback, this);

  ros::Publisher alpha_pub = pid_nh_.advertise<std_msgs::Float64>("alpha_val_manipulated", 100);


  pid_nh_.getParam("Alpha_max", Alpha_max);
  pid_nh_.getParam("Alpha_min", Alpha_min);
  pid_nh_.getParam("d", d); // Front wheel center to rear wheel line center distance
  pid_nh_.getParam("r", r); // Rear wheel center to center of line joining distance


  ros::Rate loop_rate(10);

  std_msgs::Float64 alpha_msg;

  while (ros::ok())
{
    W_t_Lock.lock();
    double Alpha_manipulated = (W_t);
    W_t_Lock.unlock();
    

    Alpha_manipulated = getMinMax(Alpha_manipulated, 45, -45);

    alpha_msg.data = Alpha_manipulated;

    alpha_pub.publish(alpha_msg);

    ros::spinOnce();
  
    loop_rate.sleep();

  }

}

int main(int argc, char** argv)
{

  WPidB *wPidB = new WPidB();
  wPidB->implementPid(argc, argv);
  delete wPidB;

}

