#ifndef STRATEGY_PLANNER
#define STRATEGY_PLANNER

#include <vector>
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <iosfwd>

static const int buffer_size = 10;
static const int loop_rate_hz = 10;

enum Navigators {
    nose_navigator = 0,
    waypoint_navigator = 1,
    lane_navigator = 2,
};



class Strategy_Planner {
    geometry_msgs::PoseStamped nose_target_, waypoint_target_, lane_target_;
    geometry_msgs::PoseStamped final_target;
    std::string which_navigator_;
    int navigators_;

public:
    void setFinalTarget(geometry_msgs::PoseStamped set_target_);
    void setWhichNavigator(std::string navigator);
    void setNoseTarget(geometry_msgs::PoseStamped proposed_nose_target_);
    void setWaypointTarget(geometry_msgs::PoseStamped proposed_waypoint_target_);
    void setLaneTarget(geometry_msgs::PoseStamped proposed_lane_target_);
    void setNavigator(int navigator);


    inline geometry_msgs::PoseStamped getNoseTarget() const {
        return nose_target_;
    }

    inline geometry_msgs::PoseStamped getWaypointTarget() const {
        return waypoint_target_;
    }

    inline geometry_msgs::PoseStamped getLaneTarget() const {
        return lane_target_;
    }

    inline geometry_msgs::PoseStamped getFinalTarget() const {
        return final_target;
    }


    inline std_msgs::String getWhichNavigator() const {
        std_msgs::String pub_str;
        pub_str.data = which_navigator_;
        return pub_str;
    }
};
#endif