#include <iostream>
#include "vx_pid.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

int manualmode=0;
using namespace std;
int bias=1400;

VxPid::VxPid() : Vx_t_lock(), Alpha_lock(), Vl_Vr_a_lock(),manualmode_lock() {

	Vr_a=0;
	Vl_a=0;
	Vx_t=0;
	Vy_t=+0.0;
	Vz_t=0.0; 
	Alpha_a=0;
	Kp_Vx=0;
	Ki_Vx=0;
	Kd_Vx=0;
	Vx_error_diff=0;
	Vx_error_old=0;
	Vx_error_integral=0;
	PWM_Duty_Cycle=0;
	vx_pid_loop_rate=0;
	//int    Vs_PID_loop_rate;

	PWM_min_percent=0; PWM_max_percent=0;
	PWM_PERIOD_TIME=0;     // in ns
	
	
}

double VxPid::getMinMax(int Cur_Var, int max, int min) {

    if (Cur_Var > max) {
		return max;
	} else {
		if (Cur_Var < min) {
			return min;
		} else {
			return Cur_Var;
		}
	}
}

void VxPid::vxTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    Vy_t_lock.lock();
    Vx_t_lock.lock();
    manualmode_lock.lock();
	Alpha_lock.lock();
	Vx_t = (msg->linear.x);
	Vy_t=(msg->linear.y);
	Vz_t=(msg->linear.z);
	//	Vs_t= (msg->linear.x )/( cos( (Alpha_a * PI)/180));
	if(std::signbit(Vz_t)) 
			manualmode = 1;
    else
			manualmode=0;
	Alpha_lock.unlock();
	Vx_t_lock.unlock();
	Vy_t_lock.unlock();
	manualmode_lock.unlock();
	

}

/*t
 void VxPid::Alpha_actual_callback(const std_msgs::Float64::ConstPtr& msg)
 {  
 ROS_INFO("\n Vs_PID_node: Alpha received \n");
 
 Alpha_lock.lock();
 Alpha_a=msg->data;
 Alpha_lock.unlock();
 
 }
 */

void VxPid::encoderCallback(const geometry_msgs::Twist::ConstPtr& msg) {
//	cout<<"@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@";
	Vl_Vr_a_lock.lock();
	Vl_a = msg->linear.x;
	Vr_a = msg->linear.y;
	Vl_Vr_a_lock.unlock();

}

void VxPid::implementPid(int argc, char** argv)
{
	

	ros::init(argc, argv, "vxpid_node");

	ros::NodeHandle nh_;
	ros::Publisher va_pub = nh_.advertise<std_msgs::Float64>("va", 100);
	ros::Subscriber Override_Subscriber = nh_.subscribe<geometry_msgs::Twist>("target_pose", 5,&VxPid::vxTargetUpdateCallback, this);
	//ros::Subscriber Alpha_Actual_Subscriber = nh_.subscribe<std_msgs::Float64>("alpha_val_actual" , 5 , Alpha_actual_callback);
	ros::Subscriber Encoder_Subscriber = nh_.subscribe<geometry_msgs::Twist>("encoders", 5, &VxPid::encoderCallback, this);
																	


	nh_.getParam("/vxpid_node/Kp_Vx", Kp_Vx);
	nh_.getParam("/vxpid_node/Ki_Vx", Ki_Vx);
	nh_.getParam("/vxpid_node/Kd_Vx", Kd_Vx);
	//nh_.getParam("/Vs_PID/Vs_PID_loop_rate", Vs_PID_loop_rate);
	nh_.getParam("/vxpid_node/PWM_min", PWM_min_percent);
	nh_.getParam("/vxpid_node/bias", bias);
	nh_.getParam("/vxpid_node/PWM_max", PWM_max_percent);
	nh_.getParam("/vxpid_node/PWM_PERIOD_TIME", PWM_PERIOD_TIME);
	nh_.getParam("/vxpid_node/vx_pid_loop_rate", vx_pid_loop_rate);
	
	// nh_.getParam("d", d); 					       // Front wheel center to rear wheel line center distance
	// nh_.getParam("r", r);					       // Rear wheel center to center of line joining distance

	
	ros::Rate loop_rate(vx_pid_loop_rate);
	
FILE *file0;
std_msgs::Float64 va_msg;

	while (ros::ok()) {
		Vl_Vr_a_lock.lock();
		Vx_t_lock.lock();
		Alpha_lock.lock();
		double Vx_error = Vx_t - Vx_a;

		float vxtprinter,vxaprinter,vxerrorprinter; 
		vxtprinter=Vx_t;
		vxaprinter=Vx_a;
		vxerrorprinter=Vx_error;
		va_msg.data=vxaprinter;
		Alpha_lock.unlock();
		Vx_t_lock.unlock();
		Vl_Vr_a_lock.unlock();
		
		
		va_pub.publish(va_msg);
		printf("Pmin: %3.3f Pmax: %3.3f ", (float)PWM_min_percent , (float)PWM_max_percent); 
		float percerror=(vxerrorprinter/vxtprinter)*100.0;
		if(vxtprinter<0.01 && vxtprinter >-0.01)
		{
			percerror=100;
		}
		printf("VT: %3.3f VA: %3.3f  error: %3.3f ", vxtprinter , vxaprinter , percerror);

		Vx_error_diff = Vx_error_old - Vx_error;
		Vx_error_integral +=  Vx_error*Ki_Vx;
		Vx_error_old = Vx_error;

		if (Vx_error_integral >= PWM_max_percent) {
			Vx_error_integral = PWM_max_percent;
		} else if (Vx_error_integral <= - PWM_max_percent) {
			Vx_error_integral = - PWM_max_percent;
		}

		PWM_Duty_Cycle = (Vx_error) * Kp_Vx + (Vx_error_integral) + (Vx_error_diff) * Kd_Vx + bias;
		//printf("Vyt is %f",Vy_t);

		if(std::signbit(Vy_t)) {
			Vx_error_integral = 0;
        } 

		printf(" P: %3.3f  I: %3.3f D: %3.3f ", (Vx_error) * Kp_Vx , (Vx_error_integral) , (Vx_error_diff) * Kd_Vx ); 

//	printf( " C_PWM: %3.3f " , PWM_Duty_Cycle);

		
		PWM_Duty_Cycle = getMinMax(PWM_Duty_Cycle, PWM_max_percent, PWM_min_percent);
int t,bit;

		

		printf(" G_PWM: %3.3f \n ", PWM_Duty_Cycle); 
		
		file0 = fopen("/dev/serial/by-id/usb-Arduino__www.arduino.cc__Arduino_Due_Prog._Port_55432333138351D09122-if00","w");
		fprintf(file0,"%d",(int)PWM_Duty_Cycle);
		fprintf(file0,"%c",'@');
		manualmode_lock.lock();
		if(manualmode==1)
		{
			fprintf(file0,"%c",'m');
		}
		else if(manualmode==0)
		{
			fprintf(file0,"%c",'a');
		}
		manualmode_lock.unlock();
		fclose(file0);
		
		ros::spinOnce();

		loop_rate.sleep();
	}
}

int main(int argc, char** argv) {
  
	VxPid * vxPid = new VxPid();

	vxPid->implementPid(argc, argv);
	
	delete vxPid;
}
