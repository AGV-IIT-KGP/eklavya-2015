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

std::mutex Vs_t_lock, Alpha_lock, Vl_Vr_a_lock;
#define PI 3.14159265
#define Vs_a ( (Vl_a+Vr_a)/2 )/( cos( (Alpha_a * PI)/180)) 

double Alpha_max;
double Alpha_min;
double d;         // Front wheel center to rear wheel line center distance
double r;		  // Rear wheel center to center of line joining distance	

double Vr_a = 0, Vl_a = 0;
double Vs_t = 0;
double Alpha_a = 0;
double Kp_Vs = 0, Ki_Vs = 0, Kd_Vs = 0;
double Vs_error_sum = 0, Vs_error_diff = 0, Vs_error_old = 0, Vs_error_integral;
double PWM_Duty_Cycle = 0;
int Vs_PID_loop_rate = 20;

int PWM_min_percent = 0, PWM_max_percent = 0;
double PWM_PERIOD_TIME = 0;     // in ns

double Get_Min_Max(int Cur_Var, int max, int min)
{
  if (Cur_Var > max)
    return max;
  else if (Cur_Var < min)
    return min;
  else
    return Cur_Var;
}

void Vs_target_update_callback(const geometry_msgs::Twist::ConstPtr& msg)
{
  ROS_INFO("Vt ");

  Vs_t_lock.lock();
  Alpha_lock.lock();
  Vs_t = (msg->linear.x) / (cos((Alpha_a * PI) / 180));
  Alpha_lock.unlock();
  Vs_t_lock.unlock();

}

void Alpha_actual_callback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("\n Vs_PID_node: Alpha received \n");

  Alpha_lock.lock();
  Alpha_a = msg->data;
  Alpha_lock.unlock();

}

void Encoder_Callback(const controls::encoder_msg::ConstPtr& msg)
{

  ROS_INFO("\n Ws_PID: Encoder Reading Received \n");

  Vl_Vr_a_lock.lock();
  Vl_a = msg->left_vel;
  Vr_a = msg->right_vel;
  Vl_Vr_a_lock.unlock();

}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "Vs_PID");

  ros::NodeHandle nh_;
  ros::Subscriber Override_Subscriber = nh_.subscribe<geometry_msgs::Twist>("target_pose", 5,
                                                                            Vs_target_update_callback);
  ros::Subscriber Alpha_Actual_Subscriber = nh_.subscribe<std_msgs::Float64>("alpha_val_actual", 5,
                                                                             Alpha_actual_callback);
  ros::Subscriber Encoder_Subscriber = nh_.subscribe<controls::encoder_msg>("encoders", 5, Encoder_Callback);

  ros::Rate loop_rate(Vs_PID_loop_rate);

  nh_.getParam("/Vs_PID/Kp_Vs", Kp_Vs);
  nh_.getParam("/Vs_PID/Ki_Vs", Ki_Vs);
  nh_.getParam("/Vs_PID/Kd_Vs", Kd_Vs);

  nh_.getParam("/Vs_PID/PWM_min", PWM_min_percent);
  nh_.getParam("/Vs_PID/PWM_max", PWM_max_percent);
  nh_.getParam("/Vs_PID/PWM_PERIOD_TIME", PWM_PERIOD_TIME);

  nh_.getParam("d", d); 					       // Front wheel center to rear wheel line center distance
  nh_.getParam("r", r);					       // Rear wheel center to center of line joining distance

  BlackLib::BlackPWM pwm_signal_pin(BlackLib::EHRPWM2A);

  pwm_signal_pin.setPeriodTime(PWM_PERIOD_TIME);

  while (ros::ok())
  {

    Vl_Vr_a_lock.lock();
    Vs_t_lock.lock();
    Alpha_lock.lock();
    double Vs_error = Vs_t - Vs_a;
    Alpha_lock.unlock();
    Vs_t_lock.unlock();
    Vl_Vr_a_lock.unlock();

    Vs_error_diff = Vs_error_old - Vs_error;
    Vs_error_sum += Vs_error;
    Vs_error_old = Vs_error;

    Vs_error_integral = Get_Min_Max(Vs_error_sum * Ki_Vs, PWM_max_percent, PWM_min_percent);

    PWM_Duty_Cycle = (Vs_error) * Kp_Vs + (Vs_error_integral) + (Vs_error_diff) * Kd_Vs;

    PWM_Duty_Cycle = Get_Min_Max(PWM_Duty_Cycle, PWM_max_percent, PWM_min_percent);

    pwm_signal_pin.setDutyPercent(PWM_Duty_Cycle);

    ros::spinOnce();
    loop_rate.sleep();

  }

}

