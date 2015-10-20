#include "w_pid.h" 
#include <iostream>

using namespace std;

WPid::WPid() :
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
	
	count_max=1000;
	count_min=-1000;

}

double WPid::getMinMax(int Cur_Var, int max, int min)
{

  if (Cur_Var > max)
    return max;
  else if (Cur_Var < min)
    return min;
  else
    return Cur_Var;
}

void WPid::encoderCallback(const controls_msgs::encoder_msg::ConstPtr& msg)
{

 
  Vl_Vr_a_lock.lock();
  Vl_a = msg->left_vel;
  Vr_a = msg->right_vel;
  Vl_Vr_a_lock.unlock();

}

void WPid::wTargetUpdateCallback(const geometry_msgs::Twist::ConstPtr& msg)
{

  W_t_Lock.lock();
  W_t = msg->angular.z;
  W_t_Lock.unlock();

}

void WPid::implementPid(int argc, char** argv)
{

  ros::init(argc, argv, "wpid_node");

  ros::NodeHandle pid_nh_;

  ros::Subscriber Override_Subscriber = pid_nh_.subscribe<geometry_msgs::Twist>("target_pose", 5,
                                                                                &WPid::wTargetUpdateCallback, this);
  ros::Subscriber Encoder_Subscriber = pid_nh_.subscribe<controls_msgs::encoder_msg>("encoders", 5, &WPid::encoderCallback,
                                                                                this);

  ros::Publisher alpha_pub = pid_nh_.advertise<std_msgs::Float64>("alpha_val_manipulated", 100);

  pid_nh_.getParam("/wpid_node/Kp_W", Kp_W);
  pid_nh_.getParam("/wpid_node/Ki_W", Ki_W);
  pid_nh_.getParam("/wpid_node/Kd_W", Kd_W);
  //pid_nh_.getParam("/wpid_node/W_PID_loop_rate", W_PID_loop_rate);

  pid_nh_.getParam("Alpha_Max", Alpha_max);
  pid_nh_.getParam("Alpha_Min", Alpha_min);
  pid_nh_.getParam("count_max", count_max);
  pid_nh_.getParam("cont_min", count_min);
  pid_nh_.getParam("d", d); // Front wheel center to rear wheel line center distance
  pid_nh_.getParam("r", r); // Rear wheel center to center of line joining distance

  pid_nh_.getParam("/wpid_node/w_pid_loop_rate", w_pid_loop_rate);

  ros::Rate loop_rate(w_pid_loop_rate);

  std_msgs::Float64 alpha_msg;

	std::cout<<std::endl;
	
  while (ros::ok())
  {
	
    Vl_Vr_a_lock.lock();
    W_t_Lock.lock();
    double W_error = W_t - W_a;
	
	printf("CMin: %3.3f CMax: %3.3f ", count_max , count_min); 
    
	printf("WT: %3.3f WA: %3.3f  error: %3.3f ", W_t, W_a, W_error); 
    
    W_t_Lock.unlock();
    Vl_Vr_a_lock.unlock();

    W_error_diff = W_error_old - W_error;
    W_error_sum += W_error;
    W_error_old = W_error;

    W_error_integral = getMinMax((W_error_sum) * Ki_W , count_max , -count_max);

    double Alpha_manipulated = (W_error) * Kp_W + (W_error_integral) + (W_error_diff) * Kd_W;


	printf(" P: %3.3f  I: %3.3f D: %3.3f ", (W_error) * Kp_W , (W_error_integral) , (W_error_diff) * Kd_W ); 


	printf( " C_Alpha: %3.3f " , Alpha_manipulated);


    Alpha_manipulated = getMinMax(Alpha_manipulated, count_max , count_min);

    alpha_msg.data = Alpha_manipulated;

    printf(" G_Alpha: %3.3f \n", Alpha_manipulated); 

    alpha_pub.publish(alpha_msg);

    ros::spinOnce();
  
    loop_rate.sleep();

  }

}

int main(int argc, char** argv)
{

  WPid *wPid = new WPid();
  wPid->implementPid(argc, argv);
  delete wPid;

}

