 #include <ros/ros.h>
#include <geometry_msgs/Twist.h>

int main(int argc, char **argv)
{
	int i=0;
	ros::init(argc, argv, "testcmdvel");
	ros::NodeHandle n;
	
	ros::Publisher cmdvelsender= n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
	ros::Rate loop_rate(10);
	int count = 0;
  while (ros::ok())
  {
   
    geometry_msgs::Twist msg;

    if(i=0)i=1;
    else if(i=1)i=0;
    msg.linear.x=i;
    msg.angular.z=i;
    
    cmdvalsender.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
  }
}
