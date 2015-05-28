#include <cmd_filtered/cmd_filtered.hpp>

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "cmd_filtered");
	std::string imu_topic;
	double global_thresh, local_thresh, low_vel;

	ros::NodeHandle nh;

	//nh.getParam("imu_topic_name", imu_topic);
	nh.getParam("global_threshold", global_thresh);
	nh.getParam("local_threshold", local_thresh);
	nh.getParam("low_velocity", low_vel);

	CMD_FILTERED cmd_filtered;



	ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", buffer_size, &CMD_FILTERED::set_instructed_omega, &cmd_filtered);
	ros::Subscriber imu_sub = nh.subscribe("imu", buffer_size, &CMD_FILTERED::set_bot_omega, &cmd_filtered);

	ros::Publisher cmd_vel_filtered_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel_filtered", buffer_size);

	double temp_thresh;
	geometry_msgs::Twist cmd_vel_filtered;

	ros::Rate loop_rate(loop_rate_hz);

	while(ros::ok())
	{
		ros::spinOnce();
		temp_thresh = cmd_filtered.instructed_omega_ - cmd_filtered.bot_omega_;

		//if(cmd_filtered.subscription_started_cmd_vel && cmd_filtered.subscription_started_imu)
		//{
			if(temp_thresh > global_thresh)
			{
				while(temp_thresh < local_thresh)
				{
					ros::spinOnce();

					cmd_vel_filtered = cmd_filtered.cmd_vel_;
					cmd_vel_filtered.linear.x = low_vel;
					
					cmd_vel_filtered_pub.publish(cmd_vel_filtered);

					temp_thresh = cmd_filtered.instructed_omega_ - cmd_filtered.bot_omega_;

					loop_rate.sleep();
				}
			}
			else
			{
				cmd_vel_filtered_pub.publish(cmd_filtered.cmd_vel_);
				
				loop_rate.sleep();
			}
		//}
	}
}