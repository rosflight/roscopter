// BSD 3-Clause License
//
// Copyright (c) 2017, James Jackson, BYU MAGICC Lab, Provo UT
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
//   list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its
//   contributors may be used to endorse or promote products derived from
//   this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


#ifndef EKF_ROS_H
#define EKF_ROS_H


#include "ekf.h"

#include <mutex>
#include <deque>
#include <vector>

#include <rclcpp/rclcpp.hpp>
// #include <rclcpp/package.h>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rosflight_msgs/msg/barometer.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <rosflight_msgs/msg/gnss.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <roscopter_msgs/msg/bool.hpp>

#ifdef UBLOX
#include "ublox/PosVelEcef.h"
#endif

#ifdef INERTIAL_SENSE
#include "inertial_sense/GPS.h"
#endif

using std::placeholders::_1;

namespace roscopter::ekf
{
class EKF_ROS : public rclcpp::Node
{
public:

  EKF_ROS();
  ~EKF_ROS();
  void init(const std::string& param_file, const std::string& param_dictionary);
  void initROS();

  void imuCallback(const sensor_msgs::msg::Imu& msg);
  void baroCallback(const rosflight_msgs::msg::Barometer& msg);
  void rangeCallback(const sensor_msgs::msg::Range& msg);
  void poseCallback(const geometry_msgs::msg::PoseStamped &msg);
  void odomCallback(const nav_msgs::msg::Odometry &msg);
  void gnssCallback(const rosflight_msgs::msg::GNSS& msg);
  void mocapCallback(const rclcpp::Time& time, const xform::Xformd &z);
  void statusCallback(const rosflight_msgs::msg::Status& msg);
  int32_t calculateTime(rclcpp::Time);

#ifdef UBLOX
  void gnssCallbackUblox(const ublox::PosVelEcefConstPtr& msg);
#endif

#ifdef INERTIAL_SENSE
  void gnssCallbackInertialSense(const inertial_sense::GPSConstPtr& msg);
#endif

  
private:
  EKF ekf_;

  rclcpp::Time last_imu_update_;

  // rclcpp::NodeHandle nh_;
  // rclcpp::NodeHandle nh_private_;
  // rclcpp::NodeHandle nh_param_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Barometer>::SharedPtr baro_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::GNSS>::SharedPtr gnss_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Range>::SharedPtr range_sub_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odometry_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr euler_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_bias_pub_;
  rclcpp::Publisher<roscopter_msgs::msg::Bool>::SharedPtr is_flying_pub_;

  sensor_msgs::msg::Imu imu_bias_msg_;
  nav_msgs::msg::Odometry odom_msg_;
  geometry_msgs::msg::Vector3Stamped euler_msg_;
  roscopter_msgs::msg::Bool is_flying_msg_;

#ifdef UBLOX
  rclcpp::Subscription ublox_gnss_sub_;
#endif

#ifdef INERTIAL_SENSE
  rclcpp::Subscription is_gnss_sub_;
#endif

  std::mutex ekf_mtx_;

  bool imu_init_ = false;
  bool truth_init_ = false;

  bool use_odom_;
  bool use_pose_;

  bool ros_initialized_ = false;
  
  bool is_flying_ = false;
  bool armed_ = false;
  rclcpp::Time time_took_off_;
  int32_t start_time_;

  Vector6d imu_;
  
  Matrix6d imu_R_;
  Matrix6d mocap_R_;
  double baro_R_;
  double range_R_;

  bool manual_gps_noise_;
  double gps_horizontal_stdev_;
  double gps_vertical_stdev_;
  double gps_speed_stdev_;

  void publishEstimates(const sensor_msgs::msg::Imu &msg);
};

}

#endif


