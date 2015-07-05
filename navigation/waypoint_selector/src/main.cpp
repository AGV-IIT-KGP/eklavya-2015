#include <waypoint_selector.hpp>

int main(int argc, char* argv[]) {
    std::string node_name = "waypoint_selector"; //launchfile-> <node .... name="waypoint_selector" />
    ros::init(argc, argv, node_name);

    int strategy;
    std::string file_name;
    ros::NodeHandle node_handle1;
    node_handle1.getParam(std::string("/") + node_name + std::string("/filename"), file_name);
    node_handle1.getParam(std::string("/") + node_name + std::string("/strategy"), strategy);

    WaypointSelector *waypoint_selector = new WaypointSelector(file_name, strategy, node_handle1);
    std_msgs::Bool nml_flag;
    ros::Rate loop_rate(loop_rate_hz);

    geometry_msgs::PoseStamped proposed_target;
    geometry_msgs::Pose2D pose2D_target;

    int flag = 0; //to make sure convert_gps_to_odom is executed only once
    while (ros::ok())
    {
        ros::spinOnce();

        // if(waypoint_selector.subscription_started_gps && waypoint_selector.subscription_started_odom && flag==0)
        // {
        //     flag = 1;
        //     waypoint_selector.convert_gps_to_odom();
        // }

        node_handle1.getParam(std::string("/") + node_name + std::string("/proximity"), waypoint_selector->proximity_);

        if(waypoint_selector->subscription_started_gps && waypoint_selector->subscription_started_odom)
        {
            pose2D_target =waypoint_selector->findTarget();
            proposed_target =waypoint_selector->convert_Pose2D_to_PoseStamped(pose2D_target);
            waypoint_selector->next_waypoint_publisher.publish(proposed_target);

            nml_flag.data = waypoint_selector->isInsideNoMansLand();
            waypoint_selector->nml_flag_publisher.publish(nml_flag);
        }

        loop_rate.sleep();
    }
}
