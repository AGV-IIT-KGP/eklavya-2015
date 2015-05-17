#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <boost/thread.hpp>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "is_test_mode");
	ros::NodeHandle n;

 	ros::Publisher confidence_pub = n.advertise<std_msgs::Bool>("test_mode", 1000);
 	//ros::Rate loop_rate(1);

 	bool is_test_mode = 0;
 	while(ros::ok())
 	{
 		std_msgs::Bool x;
 		x.data = is_test_mode;
 		ROS_INFO("is_test_mode = %d", x.data);

 		confidence_pub.publish(x);
 		ros::spinOnce();
 		//loop_rate.sleep();
boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
 		is_test_mode = !is_test_mode;
 	}
 	return 0;
}