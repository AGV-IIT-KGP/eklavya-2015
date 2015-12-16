
#include <iostream>
#include "modeswitcher.h"
#include <cmath>

using namespace ros;
using namespace std;

ModeSwitcher::ModeSwitcher() :
	Vl_Vr_a_lock() , finaltwist(), Vx_Xbox_lock() ,Vz_Xbox_lock(), W_Xbox_lock() , Vx_planner_lock() , W_planner_lock() , xbox_flag_lock(),Vy_Xbox_lock(),finalvt()
{

	d=0.9;//distance between steering and the back tires in meters

	maxalpha=45 ; minalpha = -45;

	alpha=0;

	W_xbox=0;

	Vx_Xbox=0;
	Vy_Xbox=0.0;
	Vz_Xbox=+0.0;

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
		Vz_Xbox_lock.lock();
		Vz_Xbox=+0.0;
		Vz_Xbox_lock.unlock();
//		std::cout<< "***************" <<std::endl;
		xbox_flag_lock.lock();
		xboxflag=0;
		xbox_flag_lock.unlock();
		Vy_Xbox_lock.lock();
        Vy_Xbox = - 0.0;
        Vy_Xbox_lock.unlock();

        Vy_Xbox_lock.lock();
        Vy_Xbox = - 0.0;
        Vy_Xbox_lock.unlock();

        	Vx_planner_lock.lock();
		Vx_Planner=0.0;
		Vx_planner_lock.unlock();
		W_planner_lock.lock();
		W_Planner=0.0;
		W_planner_lock.unlock();

		return;
	}

	else if(manual_button==1 && auto_button==0)
	{
		Vz_Xbox_lock.lock();
		Vz_Xbox=-0.0;
		Vz_Xbox_lock.unlock();
		xbox_flag_lock.lock();
//		std::cout<< "---------------" <<std::endl;

		xboxflag=1;
		xbox_flag_lock.unlock();
		Vy_Xbox_lock.lock();
	        Vy_Xbox = - 0.0;
       		Vy_Xbox_lock.unlock();

       		Vx_Xbox_lock.lock();
		Vx_Xbox=0.0;
		Vx_Xbox_lock.unlock();
		W_Xbox_lock.lock();
		W_xbox=0.0;
		W_Xbox_lock.unlock();


	}
				xbox_flag_lock.lock();
                int temp_flag1=xboxflag;
                xbox_flag_lock.unlock();


	if (temp_flag1==1){

 		/*float val = joy->axes[5];       //Reading from the right top trigger

		float rescaled_val =1-((val + 1.0 )/2);

		Vx_Xbox_lock.lock();

		Vx_Xbox=( rescaled_val * Max_Xbox_Vx);

		Vx_Xbox_lock.unlock();*/





		float val0=joy->buttons[0],val2=joy->buttons[2];
		float val3=joy->buttons[3],val1=joy->buttons[1];
		float valaxes=joy->buttons[6];
		float valaxes2=joy->buttons[7];

		if(val2==1 && val0==0)
		{
			Vx_Xbox_lock.lock();
			Vx_Xbox+=0.1;
			Vx_Xbox_lock.unlock();
		}
		else if(val0==1 && val2==0)
		{
			Vx_Xbox_lock.lock();
			Vx_Xbox-=0.1;
			Vx_Xbox_lock.unlock();
		}
		if(val3==1)
		{
			Vx_Xbox_lock.lock();
			Vx_Xbox=0.0;
			Vx_Xbox_lock.unlock();
			W_Xbox_lock.lock();
			W_xbox=0.0;
			W_Xbox_lock.unlock();
		}
		if(val1==1)
		{
			Vy_Xbox_lock.lock();
            Vy_Xbox = - 0.0;
            Vy_Xbox_lock.unlock();
		}
		if(valaxes==1)
		{
			W_Xbox_lock.lock();
			W_xbox=W_xbox+0.15;
			W_Xbox_lock.unlock();
		}
		if(valaxes2==1)
		{
			W_Xbox_lock.lock();
			W_xbox=W_xbox-0.15;
			W_Xbox_lock.unlock();
		}

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
	ros::Publisher send_vt = nh_.advertise<std_msgs::Float64>("vt", 5);

	while(ros::ok)
	{
		Vy_Xbox = 0.0;
		ros::spinOnce();

		xbox_flag_lock.lock();
        int temp_flag=xboxflag;
		xbox_flag_lock.unlock();

		if(temp_flag==1)
		{
			cout<<"XXX"; //  << "Vx from xbox: "<< Vx_Xbox <<" Wx from xbox: "<< W_xbox<<std::endl;
			cout<<endl;

				Vx_Xbox_lock.lock();
			 	Vy_Xbox_lock.lock();
			 	Vz_Xbox_lock.lock();

				finalvt.data=Vx_Xbox;
				finaltwist.linear.y=Vy_Xbox;
				finaltwist.linear.x=Vx_Xbox;
				finaltwist.linear.z=Vz_Xbox;
			//	cout;
				cout<<endl;
				Vx_Xbox_lock.unlock();
				Vy_Xbox_lock.unlock();
				Vz_Xbox_lock.unlock();
				W_Xbox_lock.lock();
				finaltwist.angular.z= W_xbox*Vx_Xbox;
			//	cout;
				cout<<endl;
				W_Xbox_lock.unlock();

				send_twist.publish(finaltwist);
				send_vt.publish(finalvt);
		}

		else if(temp_flag==0)
		{
			cout<<"PPP"; //<<" Vx from planner: "<< Vx_Planner <<" W from planner: "<<W_Planner <<std::endl;

				Vx_planner_lock.lock();
					finaltwist.linear.x=Vx_Planner;
					finalvt.data=Vx_Planner;
			//		cout;
                                cout<<endl;

				Vx_planner_lock.unlock();

				W_planner_lock.lock();
					finaltwist.angular.z=W_Planner;
					finaltwist.linear.x=Vx_Planner;
                //    cout;
					finaltwist.linear.y=Vy_Xbox;
					finaltwist.linear.z=Vz_Xbox;
					cout<<endl;

				W_planner_lock.unlock();
				send_twist.publish(finaltwist);
				send_vt.publish(finalvt);
		}


		loop_rate.sleep();
	}
 }



int main(int argc, char** argv)
{

	ModeSwitcher * layer = new ModeSwitcher();

	layer->publish(argc, argv);

	delete layer;
}
