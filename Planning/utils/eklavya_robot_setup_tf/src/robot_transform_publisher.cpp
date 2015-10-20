#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_publisher");
  ros::NodeHandle n;

  ros::Rate r(100);

  tf::TransformBroadcaster blink_laser_broadcaster, blink_bfootprint_broadcaster;

  while(n.ok()){
    blink_laser_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(1.06, 0.0, 0.441)),
        ros::Time::now(),"base_link", "laser"));
    blink_bfootprint_broadcaster.sendTransform(
      tf::StampedTransform(
        tf::Transform(tf::Quaternion(0, 0, 0, 1), tf::Vector3(0.0, 0.0, -0.20)),
        ros::Time::now(),"base_link", "base_footprint"));
    r.sleep();
  }
}
