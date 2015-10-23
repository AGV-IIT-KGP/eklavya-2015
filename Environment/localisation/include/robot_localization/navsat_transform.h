/*
 * Copyright (c) 2015, Charles River Analytics, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 * notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 * copyright notice, this list of conditions and the following
 * disclaimer in the documentation and/or other materials provided
 * with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived
 * from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

#include <tf/transform_datatypes.h>

#include <Eigen/Dense>

namespace RobotLocalization
{
  class NavSatTransform
  {
    public:

      //! @brief Constructor
      //!
      NavSatTransform();

      //! @brief Destructor
      //!
      ~NavSatTransform();

      //! @brief Main run loop
      //!
      void run();

    private:

      //! @brief Whether or not we broadcast the UTM transform
      //!
      bool broadcastUtmTransform_;

      //! @brief Parameter that specifies the magnetic decliation for the robot's
      //! environment.
      //!
      double magneticDeclination_;

      //! @brief Stores the yaw we need to compute the transform
      //!
      double utmOdomTfYaw_;

      //! @brief Whether or not the GPS fix is usable
      //!
      bool hasGps_;

      //! @brief Signifies that we have an odometry message
      //!
      bool hasOdom_;

      //! @brief Signifies that we have received an IMU message
      //!
      bool hasImu_;

      //! @brief Whether or not we've computed a good heading
      //!
      bool transformGood_;

      //! @brief Whether or not we have new GPS data
      //!
      //! We only want to compute and broadcast our transformed GPS
      //! data if it's new. This variable keeps track of that.
      //!
      bool gpsUpdated_;

      //! @brief Whether or not we have new odometry data
      //!
      //! If we're creating filtered GPS messages, then we only
      //! want to broadcast them when new odometry data arrives.
      //!
      bool odomUpdated_;

      //! @brief Timestamp of the latest good GPS message
      //!
      //! We assign this value to the timestamp of the odometry
      //! message that we output
      //!
      ros::Time gpsUpdateTime_;

      //! @brief Timestamp of the latest good odometry message
      //!
      //! We assign this value to the timestamp of the odometry
      //! message that we output
      //!
      ros::Time odomUpdateTime_;

      //! @brief IMU's yaw offset
      //!
      //! Your IMU should read 0 when facing *magnetic* north. If it
      //! doesn't, this (parameterized) value gives the offset (NOTE: if you
      //! have a magenetic declination, use the parameter setting for that).
      //!
      double yawOffset_;

      //! @brief Whether or not to report 0 altitude
      //!
      //! If this parameter is true, we always report 0 for the altitude
      //! of the converted GPS odometry message.
      //!
      bool zeroAltitude_;

      //! @brief Whether or not we publish filtered GPS messages
      //!
      bool publishGps_;

      //! @brief Frame ID of the GPS odometry output
      //!
      //! This will just match whatever your odometry message has
      //!
      std::string worldFrameId_;

      //! @brief UTM zone as determined after transforming GPS message
      //!
      std::string utmZone_;

      //! @brief Latest odometry data
      //!
      tf::Pose latestWorldPose_;

      //! @brief Latest GPS data, stored as UTM coords
      //!
      tf::Pose latestUtmPose_;

      //! @brief Latest IMU orientation
      //!
      tf::Quaternion latestOrientation_;

      //! @brief Covariance for most recent GPS/UTM data
      //!
      Eigen::MatrixXd latestUtmCovariance_;

      //! @brief Covariance for most recent odometry data
      //!
      Eigen::MatrixXd latestOdomCovariance_;

      //! @brief Holds the UTM->odom transform
      //!
      tf::Transform utmWorldTransform_;

      //! @brief Holds the odom->UTM transform for filtered GPS broadcast
      //!
      tf::Transform utmWorldTransInverse_;

      //! @brief Callback for the odom data
      //!
      void odomCallback(const nav_msgs::OdometryConstPtr& msg);

      //! @brief Callback for the GPS fix data
      //!
      void gpsFixCallback(const sensor_msgs::NavSatFixConstPtr& msg);

      //! @brief Callback for the IMU data
      //!
      void imuCallback(const sensor_msgs::ImuConstPtr& msg);

      //! @brief Computes the transform from the UTM frame to the
      //! odom frame
      //!
      void computeTransform();

      //! @brief Prepares the GPS odometry message before sending
      //!
      bool prepareGpsOdometry(nav_msgs::Odometry &gpsOdom);

      //! @brief Converts the odometry data back to GPS and broadcasts it
      //!
      bool prepareFilteredGps(sensor_msgs::NavSatFix &filteredGps);
  };
}
