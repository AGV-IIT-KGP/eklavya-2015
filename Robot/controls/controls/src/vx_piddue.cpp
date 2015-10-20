#include <iostream>
#include "vx_pid.h"
#include "BlackUART.h"

using namespace std;


VxPid::VxPid() :
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

double VxPid::getMinMax(int Cur_Var, int max, int min)
{

  if (Cur_Var > max)
    return max;
  else if (Cur_Var < min)
    return min;
  else
    return Cur_Var;
}

void VxPid::vxTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

  Vx_t_lock.lock();
  Alpha_lock.lock();
  Vx_t = (msg->linear.x);
  //	Vs_t= (msg->linear.x )/( cos( (Alpha_a * PI)/180));
  Alpha_lock.unlock();
  Vx_t_lock.unlock();

}

/*
 void VxPid::Alpha_actual_callback(const std_msgs::Float64::ConstPtr& msg)
 {  
 ROS_INFO("\n Vs_PID_node: Alpha received \n");
 
 Alpha_lock.lock();
 Alpha_a=msg->data;
 Alpha_lock.unlock();
 
 }
 */

void VxPid::encoderCallback(const controls_msgs::encoder_msg::ConstPtr& msg)
{
  Vl_Vr_a_lock.lock();
  Vl_a = msg->left_vel;
  Vr_a = msg->right_vel;
  Vl_Vr_a_lock.unlock();

}

void VxPid::implementPid(int argc, char** argv)
{
	

	ros::init(argc, argv, "vxpid_node");

	ros::NodeHandle nh_;
	ros::Subscriber Override_Subscriber = nh_.subscribe<geometry_msgs::Twist>("target_pose", 5,
																		&VxPid::vxTargetUpdateCallback, this);
	//ros::Subscriber Alpha_Actual_Subscriber = nh_.subscribe<std_msgs::Float64>("alpha_val_actual" , 5 , Alpha_actual_callback);
	ros::Subscriber Encoder_Subscriber = nh_.subscribe<controls_msgs::encoder_msg>("encoders", 5, &VxPid::encoderCallback,
																		this);


	BlackLib::BlackUART  Uart1(BlackLib::UART1,
                               BlackLib::Baud9600,
                               BlackLib::ParityEven,
                               BlackLib::StopOne,
                               BlackLib::Char8 );

	Uart1.open( BlackLib::ReadWrite | BlackLib::NonBlock );
	std::string writeToUart1;
    std::ostringstream os1;

	nh_.getParam("/vxpid_node/Kp_Vx", Kp_Vx);
	nh_.getParam("/vxpid_node/Ki_Vx", Ki_Vx);
	nh_.getParam("/vxpid_node/Kd_Vx", Kd_Vx);
	//nh_.getParam("/Vs_PID/Vs_PID_loop_rate", Vs_PID_loop_rate);
	nh_.getParam("/vxpid_node/PWM_min", PWM_min_percent);
	nh_.getParam("/vxpid_node/PWM_max", PWM_max_percent);
	nh_.getParam("/vxpid_node/PWM_PERIOD_TIME", PWM_PERIOD_TIME);
	nh_.getParam("/vxpid_node/vx_pid_loop_rate", vx_pid_loop_rate);
	
	// nh_.getParam("d", d); 					       // Front wheel center to rear wheel line center distance
	// nh_.getParam("r", r);					       // Rear wheel center to center of line joining distance

	
	ros::Rate loop_rate(vx_pid_loop_rate);


  while (ros::ok())
  {
    Vl_Vr_a_lock.lock();
    Vx_t_lock.lock();
    Alpha_lock.lock();
    double Vx_error = Vx_t - Vx_a;

 	printf("Pmin: %3.3f Pmax: %3.3f ", PWM_min_percent , PWM_max_percent); 
    
	printf("VT: %3.3f VA: %3.3f  error: %3.3f ", Vx_t , Vx_a , Vx_error); 

    Alpha_lock.unlock();
    Vx_t_lock.unlock();
    Vl_Vr_a_lock.unlock();

    

    Vx_error_diff = Vx_error_old - Vx_error;
    Vx_error_sum += Vx_error;
    Vx_error_old = Vx_error;

    Vx_error_integral = getMinMax ( Vx_error_sum * Ki_Vx , PWM_max_percent, -PWM_max_percent);

    PWM_Duty_Cycle = (Vx_error) * Kp_Vx + (Vx_error_integral) + (Vx_error_diff) * Kd_Vx;

	printf(" P: %3.3f  I: %3.3f D: %3.3f ", (Vx_error) * Kp_Vx , (Vx_error_integral) , (Vx_error_diff) * Kd_Vx ); 


	printf( " C_PWM: %3.3f " , PWM_Duty_Cycle);

    PWM_Duty_Cycle = getMinMax(PWM_Duty_Cycle, PWM_max_percent, PWM_min_percent);
	
	
	os1.str("");
    os1.clear();
    os1 << (int) PWM_Duty_Cycle << "\n";
    writeToUart1 = os1.str();
    Uart1 << writeToUart1;
	usleep(1000);
   
	
	printf(" G_PWM: %3.3f \n ", PWM_Duty_Cycle); 
    ros::spinOnce();

    loop_rate.sleep();

  }
}

int main(int argc, char** argv)
{
	
  
	VxPid * vxPid = new VxPid();

	vxPid->implementPid(argc, argv);
	
	delete vxPid;

}
