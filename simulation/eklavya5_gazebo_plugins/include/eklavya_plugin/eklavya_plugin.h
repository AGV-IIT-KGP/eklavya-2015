#ifndef GAZEBO_ROS_CREATE_H
#define GAZEBO_ROS_CREATE_H

#include "gazebo/physics/physics.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/sensors/SensorTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/physics/Joint.hh"

#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include <tf/transform_broadcaster.h>
#include <ros/ros.h>

namespace gazebo
{
  class EklavyaPlugin : public ModelPlugin
  {
    public: 
      EklavyaPlugin();
      virtual ~EklavyaPlugin();
          
      virtual void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf );

      virtual void UpdateChild();
      
  
    private:

      void OnContact(const std::string &name, const physics::Contact &contact);
      void OnCmdVel( const geometry_msgs::TwistConstPtr &msg);


      void publishOdometry(common::Time time_now);
      /// Parameters
      std::string node_namespace_;
      std::string bl_joint_name_;
      std::string br_joint_name_;
      std::string fl_joint_name_;
      std::string fr_joint_name_;
      std::string base_geom_name_;

      /// Separation between the wheels
      float wheel_sep_;

      /// Diameter of the wheels
      float wheel_diam_;

      ///Torque applied to the wheels
      float torque_;

      ros::NodeHandle *rosnode_;
  
      ros::Publisher sensor_state_pub_;
      ros::Publisher odom_pub_;
      ros::Publisher joint_state_pub_;
  
      ros::Subscriber cmd_vel_sub_;

      physics::WorldPtr world_;
      physics::ModelPtr model_;
      sensors::SensorPtr parent_sensor_;

      /// Speeds of the wheels
      float *wheel_speed_;

      // Simulation time of the last update
      common::Time prev_update_time_;
      common::Time last_cmd_vel_time_;

      float odom_pose_[3];
      float odom_vel_[3];
      float odom_dr_, odom_da_;
      common::Time prev_odom_time_;
      int odom_pub_counter_;

      bool set_joints_[4];
      physics::JointPtr joints_[4];
      physics::CollisionPtr base_geom_;
      
      tf::TransformBroadcaster transform_broadcaster_;
      sensor_msgs::JointState js_;

      void spin();
      boost::thread *spinner_thread_;

      // Pointer to the update event connection
      event::ConnectionPtr updateConnection;

      bool kill_sim;

      physics::JointController *j2_controller;
  };
}
#endif
