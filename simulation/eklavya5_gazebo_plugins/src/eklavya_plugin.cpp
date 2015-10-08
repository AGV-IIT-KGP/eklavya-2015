/*
 * Copyright (c) 2012, Clearpath Robotics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of Clearpath Robotics, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * Desc: Gazebo 1.x plugin for Eklavya 2015
 * Adapted from the Husky plugin
 * Author: Jit Ray Chowdhury
 */ 

#include <boost/thread.hpp>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#include <eklavya_plugin/eklavya_plugin.h>

#include <ros/time.h>

using namespace gazebo;

enum {BL= 0, BR=1, F=2};

double steer_angle = 0.0;

EklavyaPlugin::EklavyaPlugin()
{
  kill_sim = false;
  this->spinner_thread_ = new boost::thread( boost::bind( &EklavyaPlugin::spin, this) );

  wheel_speed_ = new float[3];
  wheel_speed_[BL] = 0.0;
  wheel_speed_[BR] = 0.0;
  wheel_speed_[F] = 0.0;
  
  set_joints_[0] = false;
  set_joints_[1] = false;
  set_joints_[2] = false;
  set_joints_[3] = false;
  
  joints_[0].reset();
  joints_[1].reset();
  joints_[2].reset();
  joints_[3].reset();
}

EklavyaPlugin::~EklavyaPlugin()
{
  event::Events::DisconnectWorldUpdateBegin(this->updateConnection);

  rosnode_->shutdown();
  kill_sim = true;
  this->spinner_thread_->join();
  delete this->spinner_thread_;
  delete [] wheel_speed_;
  delete rosnode_;
}
    
void EklavyaPlugin::Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf )
{
  this->model_ = _parent;
  this->world_ = this->model_->GetWorld();

  this->node_namespace_ = "";
  if (_sdf->HasElement("robotNamespace"))
    this->node_namespace_ = _sdf->GetElement("robotNamespace")->Get<std::string>() + "/";

  //not sure
  bl_joint_name_ = "LW-Chassis";//"backLeftJoint";
  if (_sdf->HasElement("LW-Chassis")) //backLeftJoint"))
    bl_joint_name_ = _sdf->GetElement("LW-Chassis")->Get<std::string>();//backLeftJoint")->Get<std::string>();

  br_joint_name_ = "RW-Chassis";//backRightJoint";
  if (_sdf->HasElement("RW-Chassis"))
    br_joint_name_ = _sdf->GetElement("RW-Chassis")->Get<std::string>();

  fl_joint_name_ = "Steerring Wheel";//frontLeftJoint";
  if (_sdf->HasElement("Steerring Wheel"))
    fl_joint_name_ = _sdf->GetElement("Steerring Wheel")->Get<std::string>();

  /*fr_joint_name_ = "frontRightJoint";
  if (_sdf->HasElement("frontRightJoint"))
    fr_joint_name_ = _sdf->GetElement("frontRightJoint")->Get<std::string>();
  */

  wheel_sep_ = 0.68;

  if (_sdf->HasElement("wheelSeparation"))
    wheel_sep_ = _sdf->GetElement("wheelSeparation")->Get<double>();

  wheel_diam_ = 0.405; //back wheels
  if (_sdf->HasElement("wheelDiameter"))
    wheel_diam_ = _sdf->GetElement("wheelDiameter")->Get<double>();

  torque_ = 15.0;
  if (_sdf->HasElement("torque"))
    torque_ = _sdf->GetElement("torque")->Get<double>();

  base_geom_name_ = "base_link";
  if (_sdf->HasElement("baseGeom"))
    base_geom_name_ = _sdf->GetElement("baseGeom")->Get<std::string>();
  base_geom_ = model_->GetChildCollision(base_geom_name_);

  //base_geom_->SetContactsEnabled(true);

  // Get the name of the parent model
  std::string modelName = _sdf->GetParent()->Get<std::string>("name");

  // Listen to the update event. This event is broadcast every
  // simulation iteration.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
      boost::bind(&EklavyaPlugin::UpdateChild, this));
  gzdbg << "Plugin model name: " << modelName << "\n";

  if (!ros::isInitialized())
  {
    int argc = 0;
    char** argv = NULL;
    ros::init(argc, argv, "gazebo_eklavya", ros::init_options::NoSigintHandler|ros::init_options::AnonymousName);
  }

  rosnode_ = new ros::NodeHandle( node_namespace_ );

  cmd_vel_sub_ = rosnode_->subscribe("cmd_vel", 1, &EklavyaPlugin::OnCmdVel, this );

  odom_pub_ = rosnode_->advertise<nav_msgs::Odometry>("odom", 1);

  joint_state_pub_ = rosnode_->advertise<sensor_msgs::JointState>("joint_states", 1);

  js_.name.push_back( bl_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( br_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  js_.name.push_back( fl_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);

  /*js_.name.push_back( fr_joint_name_ );
  js_.position.push_back(0);
  js_.velocity.push_back(0);
  js_.effort.push_back(0);*/

  prev_update_time_ = 0;
  prev_odom_time_ = 0;
  last_cmd_vel_time_ = 0;
  odom_dr_ = 0;
  odom_da_ = 0;

  joints_[BL] = model_->GetJoint(bl_joint_name_);
  joints_[BR] = model_->GetJoint(br_joint_name_);
  joints_[F] = model_->GetJoint(fl_joint_name_);
  joints_[3] = model_->GetJoint("Chassis-Steering");
  //joints_[FR] = model_->GetJoint(fr_joint_name_);

  if (joints_[BL]) set_joints_[BL] = true;
  if (joints_[BR]) set_joints_[BR] = true;
  if (joints_[F]) set_joints_[F] = true;
  if (joints_[3]) set_joints_[3] = true;
  //if (joints_[FR]) set_joints_[FR] = true;

  //initialize time and odometry position
  prev_update_time_ = last_cmd_vel_time_ = this->world_->GetSimTime();
  odom_pose_[0] = 0.0;
  odom_pose_[1] = 0.0;
  odom_pose_[2] = 0.0;
}

void EklavyaPlugin::publishOdometry(common::Time time_now) {

  common::Time step_time = time_now - prev_odom_time_;
  prev_odom_time_ = time_now;

  // Compute odometric pose
  odom_pose_[0] += odom_dr_ * cos( odom_pose_[2] );
  odom_pose_[1] += odom_dr_ * sin( odom_pose_[2] );
  odom_pose_[2] += odom_da_;

  // Compute odometric instantaneous velocity
  odom_vel_[0] = odom_dr_ / step_time.Double();
  odom_vel_[1] = 0.0;
  odom_vel_[2] = odom_da_ / step_time.Double();

  nav_msgs::Odometry odom;
  odom.header.stamp.sec = time_now.sec;
  odom.header.stamp.nsec = time_now.nsec;
  odom.header.frame_id = "odom";
  odom.child_frame_id = "base_footprint";
  odom.pose.pose.position.x = odom_pose_[0];
  odom.pose.pose.position.y = odom_pose_[1];
  odom.pose.pose.position.z = 0;

  tf::Quaternion qt;
  qt.setRPY(0,0,odom_pose_[2]);

  odom.pose.pose.orientation.x = qt.getX();
  odom.pose.pose.orientation.y = qt.getY();
  odom.pose.pose.orientation.z = qt.getZ();
  odom.pose.pose.orientation.w = qt.getW();

  double pose_cov[36] = { 1e-3, 0, 0, 0, 0, 0,
                          0, 1e-3, 0, 0, 0, 0,
                          0, 0, 1e6, 0, 0, 0,
                          0, 0, 0, 1e6, 0, 0,
                          0, 0, 0, 0, 1e6, 0,
                          0, 0, 0, 0, 0, 1e3};

  memcpy( &odom.pose.covariance[0], pose_cov, sizeof(double)*36 );
  memcpy( &odom.twist.covariance[0], pose_cov, sizeof(double)*36 );

  odom.twist.twist.linear.x = odom_vel_[0];
  odom.twist.twist.linear.y = 0;
  odom.twist.twist.linear.z = 0;

  odom.twist.twist.angular.x = 0;
  odom.twist.twist.angular.y = 0;
  odom.twist.twist.angular.z = odom_vel_[2];

  odom_pub_.publish( odom ); 
}

void EklavyaPlugin::UpdateChild()
{
  common::Time time_now = this->world_->GetSimTime();
  common::Time step_time = time_now - prev_update_time_;
  prev_update_time_ = time_now;

  double wd, ws;
  double d_bl, d_br, d_f;
  double dr, da;

  wd = wheel_diam_;
  ws = wheel_sep_;

  d_bl = d_br = d_f = 0;
  dr = da = 0;

  // Distance travelled by front wheels
  if (set_joints_[BL])
    d_bl = step_time.Double() * (wd / 2) * joints_[BL]->GetVelocity(0);
  if (set_joints_[BR])
    d_br = step_time.Double() * (wd / 2) * joints_[BR]->GetVelocity(0);
  if (set_joints_[F])
    d_f = step_time.Double() * (wd / 2) * joints_[F]->GetVelocity(0) * cos(steer_angle); //average of cosine component
  /*if (set_joints_[FR])
    d_fr = step_time.Double() * (wd / 2) * joints_[FR]->GetVelocity(0);*/

  // Can see NaN values here, just zero them out if needed
  if (isnan(d_bl)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Eklavya plugin. NaN in d_bl. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[BL]->GetVelocity(0));
    d_bl = 0;
  }
  if (isnan(d_br)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Eklavya plugin. NaN in d_br. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[BR]->GetVelocity(0));
    d_br = 0;
  }
  if (isnan(d_f)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Eklavya plugin. NaN in d_fl. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[F]->GetVelocity(0));
    d_f = 0;
  }

  /*
  if (isnan(d_fr)) {
    ROS_WARN_THROTTLE(0.1, "Gazebo ROS Eklavya plugin. NaN in d_fr. Step time: %.2f. WD: %.2f. Velocity: %.2f", step_time.Double(), wd, joints_[FR]->GetVelocity(0));
    d_fr = 0;
  }
  */
  
  dr = (d_bl + d_br + d_f) / 3;
  da = ((d_br)/2 - (d_bl)/2) / ws; // ??

  
  odom_pub_counter_++;
  odom_dr_ += dr;
  odom_da_ += da;
  double reqd_period = 0.04;
  if (odom_pub_counter_ >= reqd_period/step_time.Double()) {
     publishOdometry(time_now);
     odom_pub_counter_ = 0;
     odom_dr_ = 0;
     odom_da_ = 0;
  }



  if (set_joints_[BL])
  {
    joints_[BL]->SetVelocity( 0, wheel_speed_[BL] / (wd/2.0) );
    joints_[BL]->SetMaxForce( 0, torque_ );
  }
  if (set_joints_[BR])
  {
    joints_[BR]->SetVelocity( 0, wheel_speed_[BR] / (wd / 2.0) );
    joints_[BR]->SetMaxForce( 0, torque_ );
  }
  if (set_joints_[F])
  {
  
    // ??
    this->model_->SetJointPosition("Chassis-Steering", steer_angle);//joints_[3]->SetJointPosition("Chassis-Steering", steer_angle);

    joints_[F]->SetVelocity( 0, wheel_speed_[F] / (0.44 / 2.0) ); //front wheel has different diameter
    joints_[F]->SetMaxForce( 0, torque_ );
    
  }
  
  /* 
  if (set_joints_[FR])
  {
    joints_[FR]->SetVelocity( 0, wheel_speed_[FR] / (wd / 2.0) );
    joints_[FR]->SetMaxForce( 0, torque_ );
  }
  */


  js_.header.stamp.sec = time_now.sec;
  js_.header.stamp.nsec = time_now.nsec;
  if (this->set_joints_[BL])
  {
    js_.position[0] = joints_[BL]->GetAngle(0).Radian();
    js_.velocity[0] = joints_[BL]->GetVelocity(0);
  }

  if (this->set_joints_[BR])
  {
    js_.position[1] = joints_[BR]->GetAngle(0).Radian();
    js_.velocity[1] = joints_[BR]->GetVelocity(0);
  }

  if (this->set_joints_[F])
  {
    js_.position[2] = joints_[F]->GetAngle(0).Radian();
    js_.velocity[2] = joints_[F]->GetVelocity(0);
  }

  /*
  if (this->set_joints_[FR])
  {
    js_.position[3] = joints_[FR]->GetAngle(0).Radian();
    js_.velocity[3] = joints_[FR]->GetVelocity(0);
  }
  */

  joint_state_pub_.publish( js_ );

  // Timeout if we haven't received a cmd in <0.1 s
  common::Time time_since_last_cmd = time_now - last_cmd_vel_time_;
  if (time_since_last_cmd.Double() > 0.1)
  {
    wheel_speed_[BL] = 0;
    wheel_speed_[BR] = 0;
    wheel_speed_[F] = 0;
    //wheel_speed_[FR] = 0;
  }
}


void EklavyaPlugin::OnCmdVel( const geometry_msgs::TwistConstPtr &msg)
{
  last_cmd_vel_time_ = this->world_->GetSimTime();
  double vr, va, d_frwheel_axle;
  vr = msg->linear.x;
  va = msg->angular.z;

  d_frwheel_axle = 1.05;

  wheel_speed_[BL] = vr - va * (wheel_sep_) / 2;
  wheel_speed_[BR] = vr + va * (wheel_sep_) / 2;
  steer_angle = atan(va*d_frwheel_axle/vr);
  wheel_speed_[F] = vr/cos(steer_angle); //vr - va * (wheel_sep_) / 2;
  //wheel_speed_[FR] = vr + va * (wheel_sep_) / 2;
}

void EklavyaPlugin::spin()
{
  while(ros::ok() && !kill_sim) ros::spinOnce();
}

GZ_REGISTER_MODEL_PLUGIN(EklavyaPlugin);

