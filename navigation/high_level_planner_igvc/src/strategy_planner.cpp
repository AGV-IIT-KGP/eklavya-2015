#include <high_level_planner_igvc/strategy_planner.hpp>


void Strategy_Planner::setNoseTarget(geometry_msgs::PoseStamped proposed_nose_target_) {
	nose_target_ = proposed_nose_target_;
}

void Strategy_Planner::setWaypointTarget(geometry_msgs::PoseStamped proposed_waypoint_target_) {
	waypoint_target_ = proposed_waypoint_target_;
}

void Strategy_Planner::setLaneTarget(geometry_msgs::PoseStamped proposed_lane_target_) {
	lane_target_ = proposed_lane_target_;
}



void Strategy_Planner::setFinalTarget(geometry_msgs::PoseStamped set_target_) {
	final_target = set_target_;
}


void Strategy_Planner::setWhichNavigator(std::string navigator) {
	which_navigator_ = navigator;
}



void Strategy_Planner::setNavigator(int navigator) {
	navigators_ = navigator;
	switch (navigator) {//see enum for int values
		case nose_navigator:
			setWhichNavigator(std::string("Nose_Navigator"));
			setFinalTarget(getNoseTarget());
			break;
		case waypoint_navigator:
			setWhichNavigator(std::string("Waypoint_Navigator"));
			setFinalTarget(getWaypointTarget());
			break;
		case lane_navigator:
			setWhichNavigator(std::string("Lane_Navigator"));
			setFinalTarget(getLaneTarget());
			break;
	}

}

