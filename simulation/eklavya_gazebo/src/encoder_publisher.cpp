#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>

static const int LOOP_RATE_HZ = 10;
static const int BUFFER_SIZE = 10;

ros::Publisher encoder_data_publisher;
ros::Subscriber joint_state_subscriber;

int main(int argc, char **argv){
    ros::init(argc,argv,"encoder_publisher");
    ros::NodeHandle node_handle;

    std::string encoder_topic_name("/encoders");
    std::string joint_state_topic_name("/joint_states");
    node_handle.getParam("encoder_topic_name", encoder_topic_name);
    node_handle.getParam("joint_state_topic_name", joint_state_topic_name);

    encoder_data_publisher = node_handle.advertise

}
