#include "c_pid.h" 
#include <iostream>

using namespace std;

CPid::CPid() :
    C_t_Lock(), C_a_Lock(), Alpha_Lock(), Vl_Vr_a_lock()
{

	W_a=0;
	W_t = 0;
	curve_t = 0;
	curve_a = 0;
	Vr_a = 0;
	Vl_a = 0;
	Vx_a = 0;
	Alpha_a = 0;
	Alpha_t = 0;
	Kp_C = 0;
	Ki_C = 0;
	Kd_C = 0;
	Alpha_max=0;
	Alpha_min=0;
	r=0; d=0;
	c_error_integral=0; c_error_diff=0; c_error_old=0;
	c_pid_loop_rate=10;
	
	count_max=1000;
	count_min=-1000;
	vDead = 0.08;
}

double CPid::getMinMax(int Cur_Var, int max, int min)
{

  if (Cur_Var > max)
    return max;
  else if (Cur_Var < min)
    return min;
  else
    return Cur_Var;
}


void CPid::encoderCallback(const geometry_msgs::Twist::ConstPtr &msg)
{

  Vl_Vr_a_lock.lock();

	  Vl_a = msg->linear.x;
	  Vr_a = msg->linear.y;
	  Vx_a = ( Vl_a + Vr_a )/2 ;
	  
	  
  Vl_Vr_a_lock.unlock();

}

void CPid::vninsCallback(const geometry_msgs::Twist::ConstPtr &msg)
{
		W_a = - msg->angular.z;
		
		if(W_a < 0.002 && W_a >- 0.002)
		{
			W_a=0;
		}
		
		C_a_Lock.lock();
		
		printf("vDead: %lf",vDead);
		if ( Vx_a < vDead && Vx_a > - vDead  ){
			curve_a = 0;		
		} else {
			Vl_Vr_a_lock.lock();  
			curve_a = W_a/Vx_a;  
			Vl_Vr_a_lock.unlock(); 
			}
		
		C_a_Lock.unlock();
}

void CPid::wTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

  double tempw,tempvx,tempvy; 
 
  
  W_t    = msg->angular.z;
  tempvx = msg->linear.x ;
	
 
/*  Vl_Vr_a_lock.lock();
  
  tempvx = Vx_a ;
  
  Vl_Vr_a_lock.unlock();
*/  
  tempvy= msg->linear.y;

  if(std::signbit(tempvy)) {
			c_error_integral = 0;
  }

 C_t_Lock.lock();
 
  if (tempvx == 0){
	  curve_t = 0;
  } 
  else {
	  
	curve_t = W_t/tempvx; 
	
}
  C_t_Lock.unlock();

}

void CPid::implementPid(int argc, char** argv)
{

  ros::init(argc, argv, "cpid_node");

  ros::NodeHandle pid_nh_;

  ros::Subscriber Override_Subscriber = pid_nh_.subscribe<geometry_msgs::Twist>("target_pose", 5, &CPid::wTargetUpdateCallback, this);
  ros::Subscriber Encoder_Subscriber = pid_nh_.subscribe<geometry_msgs::Twist>("encoders", 5, &CPid::encoderCallback,this);
  ros::Subscriber VnINS_Subscriber = pid_nh_.subscribe<geometry_msgs::Twist>("vn_ins/twist", 5, &CPid::vninsCallback,this);

  ros::Publisher alpha_pub = pid_nh_.advertise<std_msgs::Float64>("alpha_val_manipulated", 100);
  ros::Publisher ca_pub = pid_nh_.advertise<std_msgs::Float64>("ca", 100);
  ros::Publisher ct_pub = pid_nh_.advertise<std_msgs::Float64>("ct", 100);
  pid_nh_.getParam("/cpid_node/Kp_C", Kp_C);
  pid_nh_.getParam("/cpid_node/Ki_C", Ki_C);
  pid_nh_.getParam("/cpid_node/Kd_C", Kd_C);
  //pid_nh_.getParam("/cpid_node/W_PID_loop_rate", W_PID_loop_rate);

  pid_nh_.getParam("Alpha_Max", Alpha_max);
  pid_nh_.getParam("Alpha_Min", Alpha_min);
  pid_nh_.getParam("count_max", count_max);
  pid_nh_.getParam("cont_min", count_min);
  pid_nh_.getParam("d", d); // Front wheel center to rear wheel line center distance
  pid_nh_.getParam("r", r); // Rear wheel center to center of line joining distance
  pid_nh_.getParam("/cpid_node/vDead", vDead);      

  pid_nh_.getParam("/cpid_node/c_pid_loop_rate", c_pid_loop_rate);



  ros::Rate loop_rate(c_pid_loop_rate);

  std_msgs::Float64 alpha_msg , ca_msg , ct_msg;

  std::cout<<std::endl;
	
  while (ros::ok())
  {
	
    Vl_Vr_a_lock.lock();
    C_t_Lock.lock();
  
    double c_error = curve_t - curve_a;

	ca_msg.data = curve_a;
	ct_msg.data = curve_t;

	printf("\nCMin: %3.3f CMax: %3.3f ", count_max , count_min); 
    
	printf("CT: %3.3f CA: %3.3f  Cerror: %3.3f  ", curve_t, curve_a, (c_error/curve_t)*100); 
    
    C_t_Lock.unlock();
    Vl_Vr_a_lock.unlock();

    c_error_diff = c_error_old - c_error;
    c_error_integral += c_error * Ki_C;
    c_error_old = c_error;

	if (c_error_integral >= count_max){
		c_error_integral = count_max;
	} else if (c_error_integral <= - count_max){
		c_error_integral = - count_max;
	}



    c_error_integral = getMinMax( c_error_integral , count_max , -count_max);

    double Alpha_manipulated = (c_error) * Kp_C + (c_error_integral) + (c_error_diff) * Kd_C;


	printf(" P: %3.3f  I: %3.3f D: %3.3f ", (c_error) * Kp_C , (c_error_integral) , (c_error_diff) * Kd_C ); 


	printf( " C_Alpha: %3.3f " , Alpha_manipulated);


    Alpha_manipulated = getMinMax(Alpha_manipulated, count_max , count_min);

    alpha_msg.data = Alpha_manipulated;

    printf(" G_Alpha: %3.3f \n", Alpha_manipulated); 

    alpha_pub.publish(alpha_msg);
    ca_pub.publish(ca_msg);
    ct_pub.publish(ct_msg);

    ros::spinOnce();
  
    loop_rate.sleep
    ();

  }

}

int main(int argc, char** argv)
{

  CPid *cPid = new CPid();
  cPid->implementPid(argc, argv);
  delete cPid;

}

