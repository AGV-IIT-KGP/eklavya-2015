#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseStamped.h>
 
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

geometry_msgs::PoseStamped final_goal;


void target_callback(const geometry_msgs::PoseStamped &target)
{
	final_goal = target;
}
 
int main(int argc, char** argv){
   ros::init(argc, argv, "simple_navigation_goals");
 
   //tell the action client that we want to spin a thread by default
   MoveBaseClient ac("move_base", true);

   ros::NodeHandle nh;

 
   //wait for the action server to come up

   ROS_INFO("Waiting for node to start");

   while(!ac.waitForServer(ros::Duration(5.0))){
     ROS_INFO("Waiting for the move_base action server to come up");
   }
 
   move_base_msgs::MoveBaseGoal goal;

   ros::Subscriber sub_proposed_target = nh.subscribe("/waypoint_navigator/proposed_target", 10, target_callback);


 
   //we'll send a goal to the robot to move 1 meter forward
   goal.target_pose.header.frame_id = final_goal.header.frame_id;
   goal.target_pose.header.stamp = final_goal.header.stamp;
 
   goal.target_pose.pose.position.x = final_goal.pose.position.x;
   goal.target_pose.pose.position.y = final_goal.pose.position.y;
   goal.target_pose.pose.position.z = final_goal.pose.position.z;

   goal.target_pose.pose.orientation.x = final_goal.pose.orientation.x;
   goal.target_pose.pose.orientation.y = final_goal.pose.orientation.y;
   goal.target_pose.pose.orientation.z = final_goal.pose.orientation.z;
   goal.target_pose.pose.orientation.w = final_goal.pose.orientation.w;
 

   ROS_INFO("Sending goal");
   ac.sendGoal(goal);
 
   ac.waitForResult();
 
   if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
     ROS_INFO("Hooray, the base moved 1 meter forward");
   else
     ROS_INFO("The base failed to move forward 1 meter for some reason");
 
   return 0;
}