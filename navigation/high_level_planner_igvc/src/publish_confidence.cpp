#include <iostream>
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include <boost/thread.hpp>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "publish_confidence");
	ros::NodeHandle n;

 	ros::Publisher confidence_pub = n.advertise<std_msgs::Bool>("confidence", 1000);
 	//ros::Rate loop_rate(1);

 	bool is_confidence = 1;
 	while(ros::ok())
 	{
 		std_msgs::Bool x;
 		x.data = is_confidence;
 		ROS_INFO("is_confidence = %d", x.data);

 		confidence_pub.publish(x);
 		ros::spinOnce();
 		//loop_rate.sleep();
 		boost::this_thread::sleep(boost::posix_time::milliseconds(1000));
 		is_confidence = !is_confidence;
 	}
 	return 0;
}