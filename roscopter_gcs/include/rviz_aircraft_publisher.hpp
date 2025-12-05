/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2024 Jacob Moore
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef RVIZ_AIRCRAFT_PUBLISHER_HPP
#define RVIZ_AIRCRAFT_PUBLISHER_HPP

#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include "roscopter_msgs/msg/state.hpp"

namespace roscopter_gcs
{

class RvizAircraftPublisher : public rclcpp::Node
{
public:
  RvizAircraftPublisher();

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_mesh_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_aircraft_path_pub_;
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr vehicle_state_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> aircraft_tf2_broadcaster_;

  // Set up parameter handling
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
  rcl_interfaces::msg::SetParametersResult
  parameters_callback(const std::vector<rclcpp::Parameter> & parameters);
  void declare_parameters();

  void state_update_callback(const roscopter_msgs::msg::State & state);
  void update_mesh();
  void update_aircraft_history();

  roscopter_msgs::msg::State vehicle_state_;

  // Persistent rviz markers
  visualization_msgs::msg::Marker aircraft_;
  visualization_msgs::msg::Marker aircraft_history_;
  std::vector<geometry_msgs::msg::Point> aircraft_history_points_;

  int i_;
};

} // roscopter_gcs

#endif
