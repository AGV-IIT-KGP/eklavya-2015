
#include <iostream>
#include "ros/ros.h"
#include <high_level_planner_igvc/FSM.h>
#include <high_level_planner_igvc/ROSTask.h>
#include <high_level_planner_igvc/DecisionMaking.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <actionlib/client/simple_action_client.h>
#include <high_level_planner_igvc/strategy_planner.hpp>
#include <geometry_msgs/PoseStamped.h>


using namespace decision_making;


Navigators navigator = waypoint_navigator;

volatile bool is_confident, is_test_mode=1, nml_flag;
geometry_msgs::PoseStamped final_goal;

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseActionGoal> MoveBaseClient;

//move_base_msgs::MoveBaseActionGoal goal;

Strategy_Planner strategy_planner;

ros::Publisher pub_target;


FSM(High_Level_Planner)
{
    enum STATES
    {
        start,
        test_mode,
        non_test_mode,
        test_lane_navigator,
        test_waypoint_navigator,
        test_nose_navigator,
        non_test_lane_navigator,
        non_test_waypoint_navigator
    }

    FSM_START(start);
    FSM_BGN
    {
        FSM_STATE(start)
        {
            FSM_TRANSITIONS
            {
                FSM_ON_CONDITION( is_test_mode , FSM_NEXT(test_mode) );
                FSM_ON_CONDITION(!is_test_mode ,FSM_NEXT(non_test_mode));
            }
        }

        FSM_STATE(test_mode)
        {
            FSM_TRANSITIONS
            {
                FSM_ON_CONDITION(navigator == nose_navigator && is_test_mode==1, FSM_NEXT(test_nose_navigator));
                FSM_ON_CONDITION(navigator == lane_navigator && is_test_mode==1, FSM_NEXT(test_lane_navigator));
                FSM_ON_CONDITION(navigator == waypoint_navigator && is_test_mode==1, FSM_NEXT(test_waypoint_navigator));
            }           
        }

        FSM_STATE(test_nose_navigator)
        {
            FSM_CALL_TASK(noseNavigator)
            FSM_TRANSITIONS
            {
                FSM_ON_CONDITION(1, FSM_NEXT(test_nose_navigator));
            }
        }

        FSM_STATE(test_lane_navigator)
        {
            FSM_CALL_TASK(laneNavigator)
            FSM_TRANSITIONS
            {
                FSM_ON_CONDITION(1,FSM_NEXT(test_lane_navigator));
            }
        }

        FSM_STATE(test_waypoint_navigator)
        {
            FSM_CALL_TASK(waypointNavigator)
            FSM_TRANSITIONS
            {
                FSM_ON_CONDITION(1, FSM_NEXT(test_waypoint_navigator));
            }
        }


        FSM_STATE(non_test_mode)
        {
            

            FSM_TRANSITIONS
            {
                FSM_ON_CONDITION(nml_flag == 0 && is_confident == 1 && is_test_mode==0, FSM_NEXT(non_test_lane_navigator));
                FSM_ON_CONDITION(nml_flag == 0 && is_confident == 0 && is_test_mode==0, FSM_NEXT(non_test_waypoint_navigator));
                FSM_ON_CONDITION(nml_flag == 1 && is_test_mode==0, FSM_NEXT(non_test_waypoint_navigator));
            }
        }

        FSM_STATE(non_test_lane_navigator)
        {

            FSM_CALL_TASK(laneNavigator)

            FSM_TRANSITIONS
            {
                FSM_ON_CONDITION(nml_flag == 1 && is_test_mode==0, FSM_NEXT(non_test_waypoint_navigator));
                FSM_ON_CONDITION(nml_flag == 0 && is_confident == 1 && is_test_mode==0, FSM_NEXT(non_test_lane_navigator));
                FSM_ON_CONDITION(nml_flag == 0 && is_confident == 0 && is_test_mode==0, FSM_NEXT(non_test_waypoint_navigator));
            }
        }

        FSM_STATE(non_test_waypoint_navigator)
        {
            FSM_CALL_TASK(waypointNavigator)
            
            FSM_TRANSITIONS
            {
                FSM_ON_CONDITION(nml_flag == 1 && is_test_mode==0, FSM_NEXT(non_test_waypoint_navigator));
                FSM_ON_CONDITION(nml_flag == 0 && is_confident == 1 && is_test_mode==0, FSM_NEXT(non_test_lane_navigator));
                FSM_ON_CONDITION(nml_flag == 0 && is_confident == 0 && is_test_mode==0, FSM_NEXT(non_test_waypoint_navigator));
                                
            }
        }
    }

    FSM_END

}


/****************************
LocalTasks
****************************/

decision_making::TaskResult noseTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    /*MoveBaseClient ac("move_base", true);

    goal.goal.target_pose = strategy_planner.getNoseTarget();
    goal.goal_id.stamp = ros::Time::now();
    goal.header.frame_id = "odom";


    ac.sendGoal(goal);
  
    ac.waitForResult();
   
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return decision_making::TaskResult::SUCCESS();
    }
    else
    {
        ROS_INFO("No data received");
        return decision_making::TaskResult::SUCCESS();
    }*/

        return decision_making::TaskResult::SUCCESS();


}

decision_making::TaskResult laneTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    /*MoveBaseClient ac("move_base", true);

    goal.target_pose = strategy_planner.getLaneTarget();
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "base_link";


    ac.sendGoal(goal);
  
    ac.waitForResult();*/
   
    //if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    //{
        return decision_making::TaskResult::SUCCESS();
   // }
   /* else
    {
        ROS_INFO("No data received lane");
        
        return decision_making::TaskResult::SUCCESS();
    }*/
}

decision_making::TaskResult waypointTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
	/*ROS_INFO("Entered waypoint task......\n");
    MoveBaseClient ac("move_base", true);

    goal.target_pose = strategy_planner.getWaypointTarget();
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "base_link";

    ROS_INFO("x = %f, y = %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
    ac.sendGoal(goal);
  	ROS_INFO("Goal sent....");
    ac.waitForResult(ros::Duration(5.0));
   	ROS_INFO("got result....");
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        return decision_making::TaskResult::SUCCESS();
    }
    else
    {
        ROS_INFO("No data received waypoint");
        
        return decision_making::TaskResult::SUCCESS();
    }*/
    final_goal = strategy_planner.getWaypointTarget();

    ROS_INFO("final_goal.x = %f, final_goal.y = %f",final_goal.pose.position.x, final_goal.pose.position.y);

   /* move_base_msgs::MoveBaseActionGoal action_goal;

    action_goal.goal.target_pose = final_goal;
    action_goal.header.frame_id = "odom";
    action_goal.header.stamp = ros::Time::now();
    action_goal.goal_id.stamp = ros::Time::now();
    action_goal.goal_id.id = "action_goal";*/

    pub_target.publish(final_goal);
    ros::Rate loop_rate(loop_rate_hz);
    loop_rate.sleep();
        return decision_making::TaskResult::SUCCESS();
    
}

void setTestMode(std_msgs::Bool test_mode) 
{
    is_test_mode = test_mode.data;
}

void setNmlFlag(std_msgs::Bool flag)
{
    nml_flag = flag.data;
}

void setConfidence(std_msgs::Bool confidence)
{
    is_confident = confidence.data;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "fsm_high_level_planner_igvc");
    ros_decision_making_init(argc, argv);
    ros::NodeHandle nh;
    RosEventQueue eventQueue;

    //ros::Subscriber sub_nml_flag = nh.subscribe("nml_flag", buffer_size, setNmlFlag);
   // ros::Subscriber sub_confidence = nh.subscribe("confidence", buffer_size, setConfidence);
    //ros::Subscriber sub_test_mode = nh.subscribe("test_mode", buffer_size, setTestMode);
   // ros::Subscriber sub_nose_navigator_proposed_target = nh.subscribe("/nose_navigator/proposed_target", buffer_size, &Strategy_Planner::setNoseTarget, &strategy_planner);
     ros::Subscriber sub_waypoint_navigator_proposed_target = nh.subscribe("/waypoint_navigator/proposed_target", buffer_size, &Strategy_Planner::setWaypointTarget, &strategy_planner);
   // ros::Subscriber sub_lane_navigator_proposed_target = nh.subscribe("/lane_navigator/proposed_target", buffer_size, &Strategy_Planner::setLaneTarget, &strategy_planner);
     pub_target = nh.advertise<geometry_msgs::PoseStamped>("move_base_simple/goal", buffer_size);

    LocalTasks::registrate("noseNavigator", noseTask);
    LocalTasks::registrate("laneNavigator", laneTask);
    LocalTasks::registrate("waypointNavigator", waypointTask);

    ros::AsyncSpinner spinner(2);
    spinner.start();

  
    ROS_INFO("Starting fsm machine...");

    eventQueue.async_spin();
    FsmHigh_Level_Planner(NULL, &eventQueue);
    eventQueue.close();

    spinner.stop();
    return 0;

}