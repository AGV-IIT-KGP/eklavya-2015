#include <robot_localization/odom_and_imu.hpp>
#define PI 3.1415926

using namespace ros;

OdometryImuCombiner::OdometryImuCombiner() {
    count = 0;
     x = 0.0;
     y = 0.0;
    th = 0.0;
    vx = 0.0;
    vy = 0.0;
    prev_yaw=0.0;
    current_time=last_time= ros::Time::now();
}

void OdometryImuCombiner::odomCallback(const geometry_msgs::Twist msg)
{
  if (ros::ok())
{
    vr=msg.linear.y;//right_vel;
    vl=msg.linear.x;//left_vel;
    v=(vl+vr)/2;
    
  }
}

void OdometryImuCombiner::imuCallback(sensor_msgs::Imu imu_msg) {
  if (ros::ok()) {
   
   
      quat= imu_msg.orientation;
      odom.twist.twist.angular=imu_msg.angular_velocity;
      

  }
}

void OdometryImuCombiner::publishUsing(ros::Publisher& publisher) {

   current_time = ros::Time::now();
   double dt = (current_time - last_time).toSec();
   ROS_INFO("quat %f ", quat.z);

   //ROS_INFO("time %lf", dt);
   tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
   tf::Matrix3x3 m(q);

   double roll, pitch, yaw;
   m.getRPY(roll, pitch, yaw);
   double delta_th = (yaw-prev_yaw);
   if(isnan(delta_th))
    delta_th=0.0;

   ROS_INFO("yaw %lf ", yaw);

    
   vx=v*cos(delta_th);
   vy=v*sin(delta_th);

  // ROS_INFO("vx %lf", vx);

  // ROS_INFO("v %lf", v);
  //compute odometry in a typical way given the velocities of the robot
    
  double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
  double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    
    ROS_INFO("vx * cos(th) %lf ", vx * cos(delta_th));
    ROS_INFO("vx * sin(th) %lf ", vx * sin(delta_th));

   x += delta_x;
   y += delta_y;

    //ROS_INFO("h %f ", x);
    //ROS_INFO("h %f ", y);

   th += delta_th;
   tf::TransformBroadcaster odom_broadcaster;

   //since all odometry is 6DOF we'll need a quaternion created from yaw
   geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
   geometry_msgs::TransformStamped odom_trans;
   odom_trans.header.stamp = current_time;
   odom_trans.header.frame_id = "odom";
   odom_trans.child_frame_id = "base_link";

   odom_trans.transform.translation.x = x;
   odom_trans.transform.translation.y = y;
   odom_trans.transform.translation.z = 0.0;
   odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    
    //publish the message

    last_time = current_time;
    prev_yaw=yaw;
    publisher.publish(odom);
    ROS_INFO("the %f ", odom.pose.pose.position.x);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "odom_imu");

    ros::NodeHandle n;
    OdometryImuCombiner combiner;
    ros::Subscriber odom_sub=n.subscribe<geometry_msgs::Twist>("encoders",50, &OdometryImuCombiner::odomCallback, &combiner);
    //ros::Subscriber odom_sub = n.subscribe<nav_msgs::Odometry>("odom", 50, &OdometryImuCombiner::odomCallback, &combiner);
    ros::Subscriber imu_sub = n.subscribe<sensor_msgs::Imu>("vn_ins/imu", 50, &OdometryImuCombiner::imuCallback, &combiner);
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom1", 50);

    ros::Rate rate(40);

    while(ros::ok()) {
        spinOnce();
        combiner.publishUsing(odom_pub);
        rate.sleep();
    }

    return 0;
}