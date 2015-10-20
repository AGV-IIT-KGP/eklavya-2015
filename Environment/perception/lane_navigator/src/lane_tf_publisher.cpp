#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>

class PoseDrawer
{
public:
  PoseDrawer() : tf_(),  target_frame_("odom")
  {
    point_sub_.subscribe(n_, "/lane_navigator/intermediate_target", 50);
    tf_filter_ = new tf::MessageFilter<geometry_msgs::PoseStamped>(point_sub_, tf_, target_frame_, 10);
    tf_filter_->registerCallback( boost::bind(&PoseDrawer::msgCallback, this, _1) );
  } ;

private:
  message_filters::Subscriber<geometry_msgs::PoseStamped> point_sub_;
  tf::TransformListener tf_;
  tf::MessageFilter<geometry_msgs::PoseStamped> * tf_filter_;
  ros::NodeHandle n_;
  std::string target_frame_;

  //  Callback to register with tf::MessageFilter to be called when transforms are available
  void msgCallback(const boost::shared_ptr<const geometry_msgs::PoseStamped>& point_ptr)
  {
    geometry_msgs::PoseStamped point_out;
    try
    {
      std::string str="odom";
      tf_.transformPose(str, *point_ptr, point_out);

      ROS_INFO("(x:%f y:%f z:%f)\n",
             point_out.pose.position.x,
             point_out.pose.position.y,
             point_out.pose.position.z);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("Failure %s\n", ex.what()); //Print exception which was caught
    }
  };

};


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "lane_tf_publisher"); //Init ROS
  PoseDrawer pd; //Construct class
  ros::spin(); // Run until interupted
};
