
#include <iostream>
#include "ros/ros.h"
#include <high_level_planner_igvc/FSM.h>
#include <high_level_planner_igvc/ROSTask.h>
#include <high_level_planner_igvc/DecisionMaking.h>
#include <std_msgs/Bool.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <high_level_planner_igvc/strategy_planner.hpp>


using namespace decision_making;


Navigators navigator = nose_navigator;

volatile bool is_confident, is_test_mode=0, nml_flag;


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal goal;

Strategy_Planner strategy_planner;


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

			}
		}

		FSM_STATE(test_lane_navigator)
		{
			FSM_CALL_TASK(laneNavigator)
			FSM_TRANSITIONS
			{

			}
		}

		FSM_STATE(test_waypoint_navigator)
		{
			FSM_CALL_TASK(waypointNavigator)
			FSM_TRANSITIONS
			{

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
	MoveBaseClient ac("move_base", true);

    goal.target_pose = strategy_planner.getNoseTarget();
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "base_link";


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
	}



}

decision_making::TaskResult laneTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
	MoveBaseClient ac("move_base", true);

	goal.target_pose = strategy_planner.getLaneTarget();
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "base_link";


    ac.sendGoal(goal);
  
    ac.waitForResult();
   
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		return decision_making::TaskResult::SUCCESS();
	}
	else
	{
		ROS_INFO("No data received lane");
		
		return decision_making::TaskResult::SUCCESS();
	}
}

decision_making::TaskResult waypointTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{

	MoveBaseClient ac("move_base", true);

    goal.target_pose = strategy_planner.getWaypointTarget();
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.header.frame_id = "base_link";


    ac.sendGoal(goal);
  
    ac.waitForResult();
   
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
	{
		return decision_making::TaskResult::SUCCESS();
	}
	else
	{
		ROS_INFO("No data received waypoint");
		
		return decision_making::TaskResult::SUCCESS();
	}
}

void setTestMode(std_msgs::Bool test_mode) 
{
    is_test_mode = test_mode.data;
}

void setNmlFlag(std_msgs::Bool flag) {
    nml_flag = flag.data;
}

void setConfidence(std_msgs::Bool confidence) {
    is_confident = confidence.data;
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "fsm_high_level_planner_igvc");
	ros_decision_making_init(argc, argv);
	ros::NodeHandle nh;
	RosEventQueue eventQueue;

	ros::Subscriber sub_nml_flag = nh.subscribe("nml_flag", buffer_size, setNmlFlag);
    ros::Subscriber sub_confidence = nh.subscribe("confidence", buffer_size, setConfidence);
    ros::Subscriber sub_test_mode = nh.subscribe("test_mode", buffer_size, setTestMode);
   // ros::Subscriber sub_nose_navigator_proposed_target = nh.subscribe("/nose_navigator/proposed_target", buffer_size, &Strategy_Planner::setNoseTarget, &strategy_planner);
   // ros::Subscriber sub_waypoint_navigator_proposed_target = nh.subscribe("/waypoint_navigator/proposed_target", buffer_size, &Strategy_Planner::setWaypointTarget, &strategy_planner);
   // ros::Subscriber sub_lane_navigator_proposed_target = nh.subscribe("/lane_navigator/proposed_target", buffer_size, &Strategy_Planner::setLaneTarget, &strategy_planner);

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