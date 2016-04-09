#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

static const int BUFFER_SIZE = 10;

class LaneBaseToOdom {
	private:
		ros::Subscriber lane_base_link;
		ros::Publisher lane_odom_pub;
		tf::TransformListener listener;

	public:
		LaneBaseToOdom(ros::NodeHandle& nh ): listener() {
			lane_base_link = nh.subscribe("lane_navigator/intermediate_target", BUFFER_SIZE, &LaneBaseToOdom::setLaneBaseLink, this);
			lane_odom_pub = nh.advertise<geometry_msgs::PoseStamped>("lane_navigator/proposed_target", BUFFER_SIZE);
		}

		void setLaneBaseLink(geometry_msgs::PoseStamped target_base_link) {
			geometry_msgs::PoseStamped target_odom;
			try{
				listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(10.0) );
				listener.transformPose("/odom", target_base_link, target_odom);
				lane_odom_pub.publish(target_odom);
    		} catch (tf::TransformException ex) {
        		ROS_ERROR("%s",ex.what());
    		}
    	}
};

int main(int argc, char** argv){
	ros::init(argc, argv, "lane_target_in_odom");

	ros::NodeHandle nh;
	LaneBaseToOdom lane_base_to_odom(nh);

	ros::spin();
    return 0;
}
