#include <vector>
#include <fstream>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <ros/ros.h>
#include <boost/bind.hpp>
#include <cmath>
#include <iosfwd>
#include <ios>
#include <iomanip>
#include <sstream>
#include <stdlib.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float64.h>
#include <gps_common/conversions.h>
#include <sensor_msgs/Imu.h>


#define PI 3.14159265


//All distance units are in meter. Conversion has been taken care of before publishing.

static const int loop_rate_hz = 10;
static const int buffer_size = 10;
static const int altitude_preset = 60;

enum Strategy {
    greedy_selector = 0,
    sequential_selector = 1,
};

class WaypointSelector {
    std::string planner_status_;
    int strategy_;
    unsigned int num_visited_waypoints_;
    int num_of_waypoints_;
    bool inside_no_mans_land_;

    tf::Pose latestUtmPose_;
    std::string utmZone_;
    double magnetic_declination_;//TODO: make this a rosparam

    double current_yaw_north_;
    double current_yaw_odom_;

    std::ifstream waypoints_;
    sensor_msgs::NavSatFix current_gps_position_;
    nav_msgs::Odometry current_odom_position_;
    nav_msgs::Odometry current_ekf_position_;
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator current_target_ptr;
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> > gps_waypoints_;
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator last_waypoint_;
    std::vector<std::pair<geometry_msgs::Pose2D, bool> > odom_waypoints_;

    ros::Subscriber planner_status_subscriber;
    ros::Subscriber fix_subscriber;
    ros::Subscriber odom_subscriber;
    ros::Subscriber yaw_subscriber_north;
   // ros::Subscriber yaw_subscriber_odom;

    //ros::Publisher odom1_publisher;

    tf::TransformListener listener;

public:
    double proximity_;
    bool subscription_started_gps;
    bool subscription_started_odom;

    bool readWaypoints(std::ifstream& waypoints, std::vector<std::pair<sensor_msgs::NavSatFix, bool> >& gps_waypoints, int& num_of_waypoints, std::string filename);
    geometry_msgs::Pose2D interpret(sensor_msgs::NavSatFix current, sensor_msgs::NavSatFix target);
    geometry_msgs::Pose2D get_Pose2D_odom(sensor_msgs::NavSatFix current, sensor_msgs::NavSatFix target);
    geometry_msgs::Pose2D getPose2DfromGPS(sensor_msgs::NavSatFix target);
    double getMod(geometry_msgs::Point p1, geometry_msgs::Pose2D p2);
    double getModgps(sensor_msgs::NavSatFix a, sensor_msgs::NavSatFix b);
    void set_current_gps_position(const sensor_msgs::NavSatFixConstPtr& msg);
    void set_current_ekf_position(nav_msgs::Odometry subscriber_ekf);
    void set_current_yaw_north(sensor_msgs::Imu current_imu);
    //void set_current_yaw_odom(std_msgs::Float64 current_yaw_odom);
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator selectNearestWaypoint();
    std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator selectNextWaypointInSequence();
    bool reachedCurrentWaypoint(std::vector<std::pair<sensor_msgs::NavSatFix, bool> >::iterator target_ptr);
    void set_planner_status(std_msgs::String status);
    WaypointSelector(std::string file, int strategy, ros::NodeHandle& node_handle);
    virtual ~WaypointSelector();
    geometry_msgs::Pose2D findTarget();
    bool isInsideNoMansLand();

    geometry_msgs::PoseStamped convert_Pose2D_to_PoseStamped(geometry_msgs::Pose2D pose2d);
    ros::Publisher next_waypoint_publisher;
    ros::Publisher nml_flag_publisher;

};
