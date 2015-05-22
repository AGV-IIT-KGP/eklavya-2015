// Author: Chinmoy Samant
// Date: 20th May 2015

#include <cmath>
#include <boost/bind.hpp>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <tf/transform_datatypes.h>
#include <vector>
#include <fstream>
#include <iosfwd>
#include <ios>
#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

const int bot_x = 500, bot_y = 900;

const cv::Point origin(0, 480); //Wrt top left corner

class Local_To_Odom
{
private:
	int count = 0;
	geometry_msgs::Pose2D local_target;
	nav_msgs::Odometry odom_data;

	ros::Subscriber lanes_subscriber;
	ros::Subscriber odom_subscriber;


public:
	int debug=1;
	Local_To_Odom();
	ros::Publisher pub_point;
	geometry_msgs::Pose2D findTarget(cv::Mat img);
	void setTarget(const sensor_msgs::ImageConstPtr msg);
	geometry_msgs::Pose2D get_odom_target();
	geometry_msgs::PoseStamped convert_Pose2D_to_PoseStamped(geometry_msgs::Pose2D pose2d);

	void setOdom(nav_msgs::Odometry subscribed_fix);

	
};