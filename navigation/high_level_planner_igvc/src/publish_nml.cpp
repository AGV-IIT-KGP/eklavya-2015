#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <boost/thread.hpp>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "publish_nml");
	ros::NodeHandle n;

 	ros::Publisher nml_pub = n.advertise<std_msgs::Bool>("nml_flag", 1000);
 	
 	//ros::Rate loop_rate(1);

 	bool nml = 0;
 	while(ros::ok())
 	{
 		std_msgs::Bool x;
 		x.data = nml;
 		ROS_INFO("nml = %d", x.data);

 		nml_pub.publish(x);
 		ros::spinOnce();
 		//loop_rate.sleep();
		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
 		nml = !nml;
 	}
 	return 0;
}