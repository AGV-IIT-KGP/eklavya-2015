
#include <iostream>
#include "ros/ros.h"
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <std_msgs/Bool.h>

using namespace decision_making;

static const int buffer_size = 10;
static const int loop_rate_hz = 10;

enum Navigators { 
    nose_navigator = 0,
    waypoint_navigator = 1,
    lane_navigator = 2,
};
bool is_confident, is_test_mode=0, nml_flag;
int navigator=0;

FSM(high_level_planner)
{
	ROS_INFO("entered FSM, %d=is_confident, %d=nml_flag, %d=is_test_mode",is_confident,nml_flag,is_test_mode);
	FSM_STATES
	{
		start,
		test_mode,
		non_test_mode,
		test_lane_navigator,
		test_waypoint_navigator,
		test_nose_navigator,
		non_test_lane_navigator,
		non_test_waypoint_navigator,
	}

	FSM_START(start);
	FSM_BGN
	{
		//ROS_INFO("Entered FSM_BGN")
		FSM_STATE(start)
		{
			ROS_INFO("Entered start: \n");
			FSM_TRANSITIONS
			{
				FSM_ON_CONDITION(is_test_mode==1,FSM_NEXT(test_mode));
				FSM_ON_CONDITION(is_test_mode==0,FSM_NEXT(non_test_mode));
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
			ROS_INFO("Entered non_test_mode: \n");

			FSM_TRANSITIONS
			{
				FSM_ON_CONDITION(nml_flag == 0 && is_confident == 1 && is_test_mode==0, FSM_NEXT(non_test_lane_navigator));
				FSM_ON_CONDITION(nml_flag == 0 && is_confident == 0 && is_test_mode==0, FSM_NEXT(non_test_waypoint_navigator));
				FSM_ON_CONDITION(nml_flag == 1 && is_test_mode==0, FSM_NEXT(non_test_waypoint_navigator));
			}
		}

		FSM_STATE(non_test_lane_navigator)
		{
			ROS_INFO("Entered non_test_lane_navigator: \n");

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
			ROS_INFO("Entered non_test_waypoint_navigator	: \n");

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

void setNmlFlag(std_msgs::Bool flag)
{
    nml_flag = flag.data;
    ROS_INFO("nml_flag = %d", nml_flag);

}

void setConfidence(std_msgs::Bool confidence) 
{
    is_confident = confidence.data;
    ROS_INFO("is_confident = %d", is_confident);
}

/****************************
LocalTasks
****************************/
decision_making::TaskResult noseTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
	ROS_INFO("Entered the noseNavigator state.");
	return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult laneTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
	ROS_INFO("Entered the laneNavigator state.");
	return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult waypointTask(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
	ROS_INFO("Entered the waypointNavigator state.");
	return decision_making::TaskResult::SUCCESS();
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "high_level_planner");
	ros_decision_making_init(argc, argv);
	ros::NodeHandle nh;
    //ros::Rate loop_rate(loop_rate_hz);
	  RosEventQueue eventQueue;

   // ros::AsyncSpinner spinner(1);
    //spinner.start();

	ros::Subscriber sub_nml_flag = nh.subscribe("nml_flag", buffer_size, setNmlFlag);
    ros::Subscriber sub_confidence = nh.subscribe("confidence", buffer_size, setConfidence);

    LocalTasks::registrate("noseNavigator", noseTask);
    LocalTasks::registrate("laneNavigator", laneTask);
    LocalTasks::registrate("waypointNavigator", waypointTask);

//ros::spin();
     ros::AsyncSpinner spinner(1);
    spinner.start();
  
    ROS_INFO("Starting wandering machine...");
    Fsmhigh_level_planner(NULL, &eventQueue);
   // ROS_INFO("pass1\n"); 
	//ros::spin();

   // spinner.stop();
    //spinner.stop();
    return 0;

}