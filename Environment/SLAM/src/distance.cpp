#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include <sensor_msgs/Imu.h>
#include "tf/transform_datatypes.h"
#include <ros/time.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/String.h>

#define pi 3.141592

double lat=0.0,longi=0.0,dist=0.0,ang=0.0,theta=0.0;
std_msgs::Float64 yaw;
sensor_msgs::NavSatFix prop;
geometry_msgs::Point p;
geometry_msgs::Quaternion quat;

void gpsCallback(const sensor_msgs::NavSatFix msg)
{
  if (ros::ok())
{
    lat=msg.latitude;
    longi=msg.longitude;
    
  }
}

/*void targetCallback(const sensor_msgs::NavSatFix msg) //-------------
{
    if(ros::ok())
    {
        prop.latitude=msg.latitude;
        prop.longitude=msg.longitude;//long
    }
}*/

void yawCallback(const sensor_msgs::Imu msg)
{
    if(ros::ok())
    {
      
      quat=msg.orientation;
      tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
   tf::Matrix3x3 m(q);

   double roll, pitch, _yaw;
   m.getRPY(roll, pitch, _yaw);
   
   yaw.data=_yaw;



    }
}

void distance(ros::Publisher& dist_pub)
{
   double lat_rad=lat*pi/180;
   double long_rad=longi*pi/180;
   double prop_lat_rad=prop.latitude*pi/180;
   double prop_long_rad=prop.longitude*pi/180;
   double delta_phi=prop_lat_rad-lat_rad;
   double delta_lambda=prop_long_rad-long_rad;

   //haversine distance
   double a=sin(delta_phi/2)*sin(delta_phi/2)+cos(prop_lat_rad)*cos(lat_rad)*sin(delta_lambda/2)*sin(delta_lambda/2);
   double c=2*atan2(sqrt(a),sqrt(1-a));
   double dist=c*(6371000);
   
   //angle
  double th=atan2(sin(delta_lambda)*cos(prop_lat_rad),cos(lat_rad)*sin(prop_lat_rad)-sin(lat_rad)*cos(prop_lat_rad)*cos(delta_lambda));

   theta=th*180/pi;

   p.x=dist;
   p.y=theta;
   p.z=yaw.data;

   ROS_INFO("Dist = %lf",p.x);
   ROS_INFO("Theta = %lf",p.y);
   ROS_INFO("Yaw = %lf",p.z);
   //dist_pub.publish(p);

}



int main(int argc, char** argv){
    ros::init(argc, argv, "distance");

     ros::NodeHandle n;

      
    double pla,plo,la,lo;
    
    n.getParam("/distance/Proposed_Latitude", pla);
    n.getParam("/distance/Proposed_Longitude",plo);
    prop.latitude=pla;
    prop.longitude=plo;

    ROS_INFO("Proposed_Latitude = %lf",pla);
    ROS_INFO("Proposed_Longitude = %lf",plo);
    ROS_INFO(" ");
   
    ros::Subscriber gps_sub=n.subscribe<sensor_msgs::NavSatFix>("gps/fltered",50, &gpsCallback);
    ros::Subscriber yaw_sub=n.subscribe<sensor_msgs::Imu>("vn_ins/Imu", 50, &yawCallback);
    ros::Publisher dist_pub = n.advertise<geometry_msgs::Point>("dist", 50);

    ros::Rate rate(10);

    while(ros::ok()) {
        ros::spinOnce();
        distance(dist_pub);
        rate.sleep();
    }

    return 0;
}
