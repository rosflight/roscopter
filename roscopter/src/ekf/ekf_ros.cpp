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

// #include <rclcpp/package.h>

#include "ekf/state.h"
#include "ekf/ekf_ros.h"
#include "roscopter_utils/yaml.h"
#include "roscopter_utils/gnss.h"

using namespace Eigen;

namespace roscopter::ekf
{

EKF_ROS::EKF_ROS() : Node("estimator")
{
  initROS();
}

EKF_ROS::~EKF_ROS()
{}

void EKF_ROS::initROS()
{
  // std::string roscopter_path = rclcpp::package::getPath("roscopter");
  // std::string parameter_filename = nh_private_.param<std::string>("param_filename", roscopter_path + "/params/ekf.yaml");
  // std::string parameter_namespace = nh_param_.getNamespace();
  // std::string parameter_dictionary =  parameter_namespace.substr( parameter_namespace.find_last_of('/') + 1);
  // YAML::Node node = YAML::LoadFile(parameter_filename);
  // if(!node[parameter_dictionary])
  // {
  //   parameter_dictionary = "";
  // }
  
  // init(parameter_filename, parameter_dictionary);

  odometry_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
  euler_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("euler_degrees", 1);
  imu_bias_pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu_bias", 1);
  is_flying_pub_ = this->create_publisher<roscopter_msgs::msg::Bool>("is_flying", 1);

  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", 100, std::bind(&EKF_ROS::imuCallback, this, _1));
  baro_sub_ = this->create_subscription<rosflight_msgs::msg::Barometer>("baro", 100, std::bind(&EKF_ROS::baroCallback, this, _1));
  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pose", 10, std::bind(&EKF_ROS::poseCallback, this, _1));
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("reference", 10, std::bind(&EKF_ROS::odomCallback, this, _1));
  gnss_sub_ = this->create_subscription<rosflight_msgs::msg::GNSS>("gnss", 10, std::bind(&EKF_ROS::gnssCallback, this, _1));
  range_sub_ = this->create_subscription<sensor_msgs::msg::Range>("range", 10, std::bind(&EKF_ROS::rangeCallback, this, _1));

// #ifdef UBLOX
//   ublox_gnss_sub_ = this-create_subscription<>("ublox_gnss", 10, &EKF_ROS::gnssCallbackUblox, this);
// #endif
#ifdef INERTIAL_SENSE
  is_gnss_sub_ = this-create_subscription<>("is_gnss", 10, &EKF_ROS::gnssCallbackInertialSense, this);
#endif

  ros_initialized_ = true;
}

void EKF_ROS::init(const std::string &param_file, const std::string &param_dictionary)
{
  ekf_.load(param_file, param_dictionary);
  double acc_stdev, gyro_stdev;
  get_yaml_node("accel_noise_stdev", param_file, acc_stdev, param_dictionary);
  get_yaml_node("gyro_noise_stdev", param_file, gyro_stdev, param_dictionary);
  imu_R_.setZero();
  imu_R_.topLeftCorner<3,3>() = acc_stdev * acc_stdev * I_3x3;
  imu_R_.bottomRightCorner<3,3>() = gyro_stdev * gyro_stdev * I_3x3;

  double pos_stdev, att_stdev;
  get_yaml_node("position_noise_stdev", param_file, pos_stdev, param_dictionary);
  get_yaml_node("attitude_noise_stdev", param_file, att_stdev, param_dictionary);
  mocap_R_ << pos_stdev * pos_stdev * I_3x3,   Matrix3d::Zero(),
      Matrix3d::Zero(),   att_stdev * att_stdev * I_3x3;

  double baro_pressure_stdev;
  get_yaml_node("baro_pressure_noise_stdev", param_file, baro_pressure_stdev,param_dictionary);
  baro_R_ = baro_pressure_stdev * baro_pressure_stdev;

  double range_stdev;
  get_yaml_node("range_noise_stdev", param_file, range_stdev, param_dictionary);
  range_R_ = range_stdev * range_stdev;

  get_yaml_node("manual_gps_noise", param_file, manual_gps_noise_, param_dictionary);
  if (manual_gps_noise_)
  {
    get_yaml_node("gps_horizontal_stdev", param_file, gps_horizontal_stdev_, param_dictionary);
    get_yaml_node("gps_vertical_stdev", param_file, gps_vertical_stdev_, param_dictionary);
    get_yaml_node("gps_speed_stdev", param_file, gps_speed_stdev_, param_dictionary);
  }

  // Declare parameters
  bool use_mocap, use_baro, use_gnss, use_zero_vel;
  this->declare_parameter("use_mocap", use_mocap);
  this->declare_parameter("use_baro", use_baro);
  this->declare_parameter("use_gnss", use_gnss);
  this->declare_parameter("use_zero_vel", use_zero_vel);
  // Get parameters
  this->get_parameter("use_mocap", use_mocap);
  this->get_parameter("use_baro", use_baro);
  this->get_parameter("use_gnss", use_gnss);
  this->get_parameter("use_zero_vel", use_zero_vel);

  ekf_.setUseMocap(use_mocap);
  ekf_.setUseBaro(use_baro);
  ekf_.setUseGNSS(use_gnss);
  ekf_.setUseZeroVel(use_zero_vel);
  start_time_ = 0.0; // Time in seconds
}

int32_t EKF_ROS::calculateTime(rclcpp::Time time)
{
  return time.seconds() + time.nanoseconds()*1e-9;
}

void EKF_ROS::publishEstimates(const sensor_msgs::msg::Imu &msg)
{

  // Pub Odom
  odom_msg_.header = msg.header;

  const State state_est = ekf_.x();
  odom_msg_.pose.pose.position.x = state_est.p(0);
  odom_msg_.pose.pose.position.y = state_est.p(1);
  odom_msg_.pose.pose.position.z = state_est.p(2);

  odom_msg_.pose.pose.orientation.w = state_est.q.w();
  odom_msg_.pose.pose.orientation.x = state_est.q.x();
  odom_msg_.pose.pose.orientation.y = state_est.q.y();
  odom_msg_.pose.pose.orientation.z = state_est.q.z();

  odom_msg_.twist.twist.linear.x = state_est.v(0);
  odom_msg_.twist.twist.linear.y = state_est.v(1);
  odom_msg_.twist.twist.linear.z = state_est.v(2);

  odom_msg_.twist.twist.angular.x = state_est.w(0);
  odom_msg_.twist.twist.angular.y = state_est.w(1);
  odom_msg_.twist.twist.angular.z = state_est.w(2);

  odometry_pub_->publish(odom_msg_);

  // Pub Euler Attitude
  euler_msg_.header = msg.header;
  const Eigen::Vector3d euler_angles = state_est.q.euler() * 180. / M_PI;
  euler_msg_.vector.x = euler_angles(0);
  euler_msg_.vector.y = euler_angles(1);
  euler_msg_.vector.z = euler_angles(2);

  euler_pub_->publish(euler_msg_);

  // Pub Imu Bias estimate
  imu_bias_msg_.header = msg.header;

  imu_bias_msg_.angular_velocity.x = state_est.bg(0);
  imu_bias_msg_.angular_velocity.y = state_est.bg(1);
  imu_bias_msg_.angular_velocity.z = state_est.bg(2);

  imu_bias_msg_.linear_acceleration.x = state_est.ba(0);
  imu_bias_msg_.linear_acceleration.y = state_est.ba(1);
  imu_bias_msg_.linear_acceleration.z = state_est.ba(2);

  imu_bias_pub_->publish(imu_bias_msg_);

  // Only publish is_flying is true once
  if (!is_flying_)
  {
    is_flying_ = ekf_.isFlying();
    if (is_flying_)
    {
      is_flying_msg_.data = is_flying_;
      is_flying_pub_->publish(is_flying_msg_);
    }
  }
}

void EKF_ROS::imuCallback(const sensor_msgs::msg::Imu &msg)
{
  if (start_time_ == 0)
  {
    start_time_ = msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9;
  }

  Vector6d z;
  z << msg.linear_acceleration.x,
       msg.linear_acceleration.y,
       msg.linear_acceleration.z,
       msg.angular_velocity.x,
       msg.angular_velocity.y,
       msg.angular_velocity.z;

  double t = EKF_ROS::calculateTime(msg.header.stamp) - start_time_;
  ekf_.imuCallback(t, z, imu_R_);

  if(ros_initialized_)
    publishEstimates(msg);
}

void EKF_ROS::baroCallback(const rosflight_msgs::msg::Barometer& msg)
{
  const double pressure_meas = msg.pressure;
  const double temperature_meas = msg.temperature;

  if (!ekf_.groundTempPressSet())
  {
    std::cout << "Set ground pressure and temp" << std::endl;
    std::cout << "press: " << pressure_meas << std::endl;
    ekf_.setGroundTempPressure(temperature_meas, pressure_meas);
  }

  if (start_time_ == 0)
    return;

  const double t = EKF_ROS::calculateTime(msg.header.stamp) - start_time_;
  ekf_.baroCallback(t, pressure_meas, baro_R_, temperature_meas);
}

void EKF_ROS::rangeCallback(const sensor_msgs::msg::Range& msg)
{
  if (start_time_ == 0)
    return;

  const double range_meas = msg.range;
  if (range_meas < msg.max_range && range_meas > msg.min_range)
  {
    const double t = EKF_ROS::calculateTime(msg.header.stamp) - start_time_;
    ekf_.rangeCallback(t, range_meas, range_R_);
  }
}

void EKF_ROS::poseCallback(const geometry_msgs::msg::PoseStamped &msg)
{
  xform::Xformd z;
  z.arr_ << msg.pose.position.x,
          msg.pose.position.y,
          msg.pose.position.z,
          msg.pose.orientation.w,
          msg.pose.orientation.x,
          msg.pose.orientation.y,
          msg.pose.orientation.z;

  mocapCallback(msg.header.stamp, z);
}

void EKF_ROS::odomCallback(const nav_msgs::msg::Odometry &msg)
{
  xform::Xformd z;
  z.arr_ << msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z;

  mocapCallback(msg.header.stamp, z);
}

void EKF_ROS::mocapCallback(const rclcpp::Time &time, const xform::Xformd &z)
{
  if (start_time_ == 0)
    return;

  double t = EKF_ROS::calculateTime(time) - start_time_;
  ekf_.mocapCallback(t, z, mocap_R_);
}

void EKF_ROS::statusCallback(const rosflight_msgs::msg::Status &msg)
{
  if (msg.armed)
  {
    ekf_.setArmed();
  }
  else
  {
    ekf_.setDisarmed();
  }
}

void EKF_ROS::gnssCallback(const rosflight_msgs::msg::GNSS &msg)
{
  Vector6d z;
  z << msg.position[0],
       msg.position[1],
       msg.position[2],
       msg.velocity[0],
       msg.velocity[1],
       msg.velocity[2];

  // rotate covariance into the ECEF frame
  Vector6d Sigma_diag_NED;
  if (manual_gps_noise_)
  {
    Sigma_diag_NED << gps_horizontal_stdev_,
                      gps_horizontal_stdev_,
                      gps_vertical_stdev_,
                      gps_speed_stdev_,
                      gps_speed_stdev_,
                      gps_speed_stdev_;
  }
  else
  {
    Sigma_diag_NED << msg.horizontal_accuracy,
                  msg.horizontal_accuracy,
                  msg.vertical_accuracy,
                  msg.speed_accuracy,
                  msg.speed_accuracy,
                  msg.speed_accuracy;
  }

  Sigma_diag_NED = Sigma_diag_NED.cwiseProduct(Sigma_diag_NED);
  Matrix3d R_e2n = q_e2n(ecef2lla(z.head<3>())).R();

  Matrix6d Sigma_ecef;
  Sigma_ecef << R_e2n.transpose() * Sigma_diag_NED.head<3>().asDiagonal() * R_e2n, Matrix3d::Zero(),
                Matrix3d::Zero(), R_e2n.transpose() *  Sigma_diag_NED.tail<3>().asDiagonal() * R_e2n;

  if (!ekf_.refLlaSet())
  {
    // set ref lla to first gps position
    Eigen::Vector3d ref_lla = ecef2lla(z.head<3>());
    // Convert radians to degrees
    ref_lla.head<2>() *= 180. / M_PI;
    ekf_.setRefLla(ref_lla);
  }

  if (start_time_ == 0)
    return;

  double t = EKF_ROS::calculateTime(msg.header.stamp) - start_time_;
  ekf_.gnssCallback(t, z, Sigma_ecef);
}

// #ifdef UBLOX
// void EKF_ROS::gnssCallbackUblox(const ublox::msg::PosVelEcef &msg)
// {
//   if (msg.fix == ublox::msg::PosVelEcef::FIX_TYPE_2D
//       || msg.fix == ublox::msg::PosVelEcef::FIX_TYPE_3D)
//   {
//     rosflight_msgs::msg::GNSS rf_msg;
//     rf_msg.header.stamp = msg.header.stamp;
//     rf_msg.position = msg.position;
//     rf_msg.velocity = msg.velocity;
//     rf_msg.horizontal_accuracy = msg.horizontal_accuracy;
//     rf_msg.vertical_accuracy = msg.vertical_accuracy;
//     rf_msg.speed_accuracy = msg.speed_accuracy;
//     gnssCallback(boost::make_shared<rosflight_msgs::msg::GNSS>(rf_msg));
//   }
//   else
//   {
//     ROS_WARN_THROTTLE(1., "Ublox GPS not in fix");
//   }
// }
// #endif

#ifdef INERTIAL_SENSE
void EKF_ROS::gnssCallbackInertialSense(const inertial_sense::msg::GPS &msg)
{
  if (msg.fix_type == inertial_sense::GPS::GPS_STATUS_FIX_TYPE_2D_FIX
      || msg.fix_type == inertial_sense::GPS::GPS_STATUS_FIX_TYPE_3D_FIX)
  {
    rosflight_msgs::msg::GNSS rf_msg;
    rf_msg.header.stamp = msg.header.stamp;
    rf_msg.position[0] = msg.posEcef.x;
    rf_msg.position[1] = msg.posEcef.y;
    rf_msg.position[2] = msg.posEcef.z;
    rf_msg.velocity[0] = msg.velEcef.x;
    rf_msg.velocity[1] = msg.velEcef.y;
    rf_msg.velocity[2] = msg.velEcef.z;
    rf_msg.horizontal_accuracy = msg.hAcc;
    rf_msg.vertical_accuracy = msg.vAcc;
    rf_msg.speed_accuracy = 0.3;
    gnssCallback(boost::make_shared<rosflight_msgs::msg::GNSS>(rf_msg));
  }
  else
  {
    ROS_WARN_THROTTLE(1., "Inertial Sense GPS not in fix");
  }
}
#endif




}
