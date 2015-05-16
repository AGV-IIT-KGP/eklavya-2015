
#include <waypoint_navigator.hpp>

int main(int argc, char* argv[]) {
    WaypointNavigator waypoint_navigator;
    std::string yaw_sub_topic;
    std::string node_name = "waypoint_navigator";
    ros::init(argc, argv, std::string("waypoint_navigator"));
    ros::NodeHandle node_handle;

    node_handle.getParam(std::string("/") + node_name + std::string("/yaw_subscriber_topic"), yaw_sub_topic);
    ros::Subscriber next_waypoint_subscriber = node_handle.subscribe("waypoint_selector/next_waypoint", buffer_size, &WaypointNavigator::set_target_fix, &waypoint_navigator);
    ros::Subscriber current_fix_subscriber = node_handle.subscribe("vn_ins/fix", buffer_size, &WaypointNavigator::set_current_fix, &waypoint_navigator);
    ros::Subscriber current_yaw_subscriber = node_handle.subscribe(yaw_sub_topic.c_str(), buffer_size, &WaypointNavigator::set_heading, &waypoint_navigator);
    geometry_msgs::PoseStamped proposed_target;
    ros::Publisher target_publisher = node_handle.advertise<geometry_msgs::PoseStamped>("waypoint_navigator/proposed_target", buffer_size);

    ros::Rate loop_rate(loop_rate_hz);
    while (ros::ok()) {
        ros::spinOnce();
        geometry_msgs::Pose2D pose2d=waypoint_navigator.interpret();
        proposed_target=waypoint_navigator.convert_Pose2D_to_PoseStamped(pose2d);
        target_publisher.publish(proposed_target);
        loop_rate.sleep();
    }
}