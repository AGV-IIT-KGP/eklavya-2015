#include <ros/ros.h>
#include <unistd.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int32.h>
#include <cstdlib>
#include "BlackPWM.h"
#include "BlackGPIO.h"
#include <iostream>


// axes1=Left horizontal, -1 at left extreme
// axes2=Left vertical, -1 at bottom

//axes3 =Right horizontal, -1 at left extreme
//axes4 =Right vertical, -1 at bottom


  

using namespace std;
using namespace BlackLib;
int estopflag=0;
class XboxData
{
	public:
  		XboxData();
		ros::NodeHandle nh_;
	private:
  		void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  		ros::Subscriber joy_sub_;
};


XboxData::XboxData():
	nh_()
{
  	joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("joy", 10, &XboxData::joyCallback, this);
}
void XboxData::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{

  	BlackGPIO estopen(GPIO_31,output);
  	BlackPWM pwmLed2(EHRPWM2A);
  	pwmLed2.setPeriodTime(50000000);
  	pwmLed2.setDutyPercent(0.0);
  	int i;
  	float duty;

  	float val = joy->axes[4];
  	duty=(val+1)*50.0;

	if(joy->axes[1]&&duty<=80 && duty >=16)
	{
		cout<<"Current Duty Cycle= "<<100-duty;
  		cout<<endl;
        pwmLed2.setDutyPercent(100-duty);
	}
	else if(joy->axes[1]&&duty>=80)
	{
		cout<<"Current Duty Cycle= "<<0;
  		cout<<endl;
        pwmLed2.setDutyPercent(0);
	}
	else if(joy->axes[1]&&duty<=16)
	{
		cout<<"Current Duty Cycle= "<<84;
  		cout<<endl;
        pwmLed2.setDutyPercent(84);
	}
	if((joy->buttons[3]==1) && (estopflag==0))
	{
		estopflag=1;
		cout<<"E-stop Enabled";
		cout<<endl;
	}
	if((joy->buttons[2]==1) && (estopflag==1))
	{
		estopflag=0;
		cout<<"E-stop Disabled";
		cout<<endl;
	}
	if(joy->buttons[4]==1)
	{
		std_msgs::Int32 msg;
		ros::Publisher sendangle= nh_.advertise<std_msgs::Int32>("beaglelisten", 1);
		ros::Rate loop_rate(1);
		msg.data = 200;
		sendangle.publish(msg);
	}
	float dataaxes=joy->axes[2];
	if(dataaxes>=8500)
	{
		int dataaxesfin=(dataaxes-8500)/49;
		std_msgs::Int32 msg;
		ros::Publisher sendangle= nh_.advertise<std_msgs::Int32>("beaglelisten", 1);
		ros::Rate loop_rate(1);
		msg.data = dataaxesfin;
		sendangle.publish(msg);
	}
	else if(dataaxes<=-8500)
	{
		int dataaxesfin=(dataaxes+8500)/49;
		std_msgs::Int32 msg;
		ros::Publisher sendangle= nh_.advertise<std_msgs::Int32>("beaglelisten", 1);
		ros::Rate loop_rate(1);
		msg.data = dataaxesfin;
		sendangle.publish(msg);
	}
	if(estopflag==0)
	{
		estopen.setValue(low);
	}
	else if(estopflag==1)
	{
		estopen.setValue(high);
	}
}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "xbox_node");
  	XboxData myXbox;
  	ros::spin();
}
