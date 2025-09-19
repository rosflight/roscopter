/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2023 Brandon Sutherland, Jacob Moore, BYU MAGICC Lab.
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

/**
 * @file tuning_signal_generator.hpp
 *
 * @author Brandon Sutherland <brandonsutherland2@gmail.com>
 */

#include <chrono>
#include <cmath>
#include <string>

#include "signal_generator.hpp"

namespace roscopter
{
TuningSignalGenerator::TuningSignalGenerator()
    : Node("signal_generator")
    , controller_mode_(RosCopterControllerMode::NPOS_EPOS_DPOS_YAW)
    , controller_output_(ControllerOutput::CMD1)
    , signal_type_(SignalType::SQUARE)
    , publish_rate_hz_(0)
    , signal_magnitude_(0)
    , frequency_hz_(0)
    , initial_time_(0)
    , is_paused_(true)
    , paused_time_(0)
    , single_period_start_time_(0)
{
  declare_params();
  update_params();
  initial_time_ = this->get_clock()->now().seconds();

  command_publisher_ =
    this->create_publisher<roscopter_msgs::msg::ControllerCommand>("/high_level_command", 1);

  publish_timer_ =
    this->create_wall_timer(std::chrono::milliseconds(static_cast<long>(1000 / publish_rate_hz_)),
                            std::bind(&TuningSignalGenerator::publish_timer_callback, this));

  param_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&TuningSignalGenerator::param_callback, this, std::placeholders::_1));

  step_toggle_service_ = this->create_service<std_srvs::srv::Trigger>(
    "signal_generator/toggle_step_signal",
    std::bind(&TuningSignalGenerator::step_toggle_service_callback, this, std::placeholders::_1,
              std::placeholders::_2));
  reset_service_ = this->create_service<std_srvs::srv::Trigger>(
    "signal_generator/reset_signal",
    std::bind(&TuningSignalGenerator::reset_service_callback, this, std::placeholders::_1,
              std::placeholders::_2));
  pause_service_ = this->create_service<std_srvs::srv::Trigger>(
    "signal_generator/pause_signal",
    std::bind(&TuningSignalGenerator::pause_service_callback, this, std::placeholders::_1,
              std::placeholders::_2));
  start_continuous_service_ = this->create_service<std_srvs::srv::Trigger>(
    "signal_generator/start_continuous_signal",
    std::bind(&TuningSignalGenerator::start_continuous_service_callback, this,
              std::placeholders::_1, std::placeholders::_2));
  start_single_service_ = this->create_service<std_srvs::srv::Trigger>(
    "signal_generator/start_single_period_signal",
    std::bind(&TuningSignalGenerator::start_single_service_callback, this, std::placeholders::_1,
              std::placeholders::_2));
}

void TuningSignalGenerator::publish_timer_callback()
{
  update_params();

  double elapsed_time = this->get_clock()->now().seconds() - initial_time_ - paused_time_;

  // Check if only suppose to run for single period, pausing when complete
  if (abs(single_period_start_time_) > 0.01
      && (single_period_start_time_ + (1 / frequency_hz_)) <= this->get_clock()->now().seconds()) {
    single_period_start_time_ = 0;
    is_paused_ = true;
  }

  // Check if step toggle needs to be reset
  if (signal_type_ != SignalType::STEP) { step_toggled_ = false; }

  // If paused, negate passing of time but keep publishing
  if (is_paused_) { paused_time_ += 1 / publish_rate_hz_; }

  // Get value for signal
  double amplitude = signal_magnitude_ / 2;
  double center_value = 0;
  switch (controller_output_) {
    case ControllerOutput::CMD1:
      center_value = default_cmd1_;
      break;
    case ControllerOutput::CMD2:
      center_value = default_cmd2_;
      break;
    case ControllerOutput::CMD3:
      center_value = default_cmd3_;
      break;
    case ControllerOutput::CMD4:
      center_value = default_cmd4_;
      break;
  }

  center_value += amplitude;

  double signal_value = 0;
  switch (signal_type_) {
    case SignalType::STEP:
      signal_value = get_step_signal(step_toggled_, amplitude, center_value);
      break;
    case SignalType::SQUARE:
      signal_value = get_square_signal(elapsed_time, amplitude, frequency_hz_, center_value);
      break;
    case SignalType::SAWTOOTH:
      signal_value = get_sawtooth_signal(elapsed_time, amplitude, frequency_hz_, center_value);
      break;
    case SignalType::TRIANGLE:
      signal_value = get_triangle_signal(elapsed_time, amplitude, frequency_hz_, center_value);
      break;
    case SignalType::SINE:
      signal_value = get_sine_signal(elapsed_time, amplitude, frequency_hz_, center_value);
      break;
  }

  // Creates message with default values
  roscopter_msgs::msg::ControllerCommand command_message;
  command_message.header.stamp = this->get_clock()->now();
  command_message.mode = static_cast<uint8_t>(controller_mode_);
  command_message.cmd_valid = true;

  command_message.cmd1 = default_cmd1_;;
  command_message.cmd2 = default_cmd2_;;
  command_message.cmd3 = default_cmd3_;;
  command_message.cmd4 = default_cmd4_;;

  // Publish message
  switch (controller_output_) {
    case ControllerOutput::CMD1:
      command_message.cmd1 = signal_value;
      break;
    case ControllerOutput::CMD2:
      command_message.cmd2 = signal_value;
      break;
    case ControllerOutput::CMD3:
      command_message.cmd3 = signal_value;
      break;
    case ControllerOutput::CMD4:
      command_message.cmd4 = signal_value;
      break;
  }
  command_publisher_->publish(command_message);
}

rcl_interfaces::msg::SetParametersResult
TuningSignalGenerator::param_callback(const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  bool do_reset = false;
  for (const auto & param : params) {
    if (param.get_name() == "controller_mode") {
      do_reset = true;
    } else if (param.get_name() == "signal_type") {
      do_reset = true;
      if (signal_types_.count(param.as_string()) == 0) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Parameter signal_type set to an invalid value: "
                           << param.as_string() << "! Rejecting param update.");
        do_reset = false;
        result.successful = false;
      }
    }
  }

  if (do_reset) {
    // Only reset if all params were valid
    reset();
  }

  return result;
}

bool TuningSignalGenerator::step_toggle_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (signal_type_ != SignalType::STEP) {
    res->success = false;
    res->message = "Service valid only for step type signal";
    return true;
  }

  if (step_toggled_) {
    step_toggled_ = false;
  } else {
    step_toggled_ = true;
  }

  res->success = true;
  return true;
}

bool TuningSignalGenerator::reset_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  reset();
  res->success = true;
  return true;
}

bool TuningSignalGenerator::pause_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (signal_type_ == SignalType::STEP) {
    res->success = false;
    res->message = "Service not valid for step type signal";
    return true;
  }

  is_paused_ = true;
  single_period_start_time_ = 0;

  res->success = true;
  return true;
}

bool TuningSignalGenerator::start_continuous_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (signal_type_ == SignalType::STEP) {
    res->success = false;
    res->message = "Service not valid for step type signal";
    return true;
  }

  is_paused_ = false;
  single_period_start_time_ = 0;

  res->success = true;
  return true;
}

bool TuningSignalGenerator::start_single_service_callback(
  const std_srvs::srv::Trigger::Request::SharedPtr & req,
  const std_srvs::srv::Trigger::Response::SharedPtr & res)
{
  if (signal_type_ == SignalType::STEP) {
    res->success = false;
    res->message = "Service not valid for step type signal";
    return true;
  }

  is_paused_ = false;
  single_period_start_time_ = this->get_clock()->now().seconds();

  res->success = true;
  return true;
}

double TuningSignalGenerator::get_step_signal(bool step_toggled, double amplitude,
                                              double center_value)
{
  return (step_toggled - 0.5) * 2 * amplitude + center_value;
}

double TuningSignalGenerator::get_square_signal(double elapsed_time, double amplitude,
                                                double frequency, double center_value)
{
  // amplitude * (1 to -1 switching value) + center value
  return amplitude * ((static_cast<int>(elapsed_time * frequency * 2) % 2) * 2 - 1) + center_value;
}

double TuningSignalGenerator::get_sawtooth_signal(double elapsed_time, double amplitude,
                                                  double frequency, double center_value)
{
  // slope * elapsed_time - num_cycles * offset_per_cycle + center value
  return 2 * amplitude
    * (elapsed_time * frequency - static_cast<int>(elapsed_time * frequency) - 0.5)
    + center_value;
}

double TuningSignalGenerator::get_triangle_signal(double elapsed_time, double amplitude,
                                                  double frequency, double center_value)
{
  // (1 to -1 switching value) * sawtooth_at_twice_the_rate + center_value
  return -((static_cast<int>(elapsed_time * frequency * 2) % 2) * 2 - 1) * 2 * amplitude
    * (2 * elapsed_time * frequency - static_cast<int>(2 * elapsed_time * frequency) - 0.5)
    + center_value;
}

double TuningSignalGenerator::get_sine_signal(double elapsed_time, double amplitude,
                                              double frequency, double center_value)
{
  return -cos(elapsed_time * frequency * 2 * M_PI) * amplitude + center_value;
}

void TuningSignalGenerator::declare_params()
{
  // Declare the parameters and the parameter descriptors
  auto controller_mode_param_desc = rcl_interfaces::msg::ParameterDescriptor();
  controller_mode_param_desc.type = rclcpp::PARAMETER_INTEGER;
  // controller_mode_param_desc.description =
  //   "Output mode on the /high_level_command topic. See the roscopter_msgs/msg/ControllerCommand for a description of the modes.";
  std::ostringstream desc;
  desc << "Output mode on the /high_level_command topic. From the roscopter_msgs/msg/ControllerOutput message definition: \n";
  desc << "MODE_NPOS_EPOS_DPOS_YAW = 0\n";
  desc << "MODE_NVEL_EVEL_DPOS_YAWRATE = 1\n";
  desc << "MODE_NACC_EACC_DACC_YAWRATE = 2\n";
  desc << "MODE_NVEL_EVEL_DVEL_YAWRATE = 3\n";
  desc << "MODE_NPOS_EPOS_DVEL_YAW = 4\n";
  desc << "MODE_ROLL_PITCH_YAW_THROTTLE = 5\n";
  desc << "MODE_ROLL_PITCH_YAWRATE_THROTTLE = 6\n";
  desc << "MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE = 7\n";
  desc << "MODE_PASS_THROUGH_TO_MIXER = 8\n";
  desc << "MODE_ROLL_PITCH_YAW_THRUST_TO_MIXER = 9\n";
  desc << "MODE_ROLL_PITCH_YAWRATE_THRUST_TO_MIXER = 10\n";
  desc << "MODE_ROLLRATE_PITCHRATE_YAWRATE_THRUST_TO_MIXER = 11";
  controller_mode_param_desc.description = desc.str();
  controller_mode_param_desc.integer_range = {rcl_interfaces::msg::IntegerRange().set__from_value(0).set__to_value(10)};
  this->declare_parameter("controller_mode", 0, controller_mode_param_desc);

  auto controller_output_param_desc = rcl_interfaces::msg::ParameterDescriptor();
  controller_output_param_desc.type = rclcpp::PARAMETER_INTEGER;
  controller_output_param_desc.description = "Channel the output command gets sent on. Options: 1,2,3,4.";
  controller_output_param_desc.integer_range = {rcl_interfaces::msg::IntegerRange().set__from_value(1).set__to_value(4)};
  this->declare_parameter("controller_output", 1, controller_output_param_desc);

  auto sig_type_param_desc = rcl_interfaces::msg::ParameterDescriptor();
  sig_type_param_desc.type = rclcpp::PARAMETER_STRING;
  sig_type_param_desc.description = "Type of the output signal. Options: step, square, sawtooth, triangle, sine.";
  this->declare_parameter("signal_type", "step", sig_type_param_desc);
  this->declare_parameter("publish_rate_hz", 100.0);
  this->declare_parameter("signal_magnitude", 1.0);
  this->declare_parameter("frequency_hz", 0.2);
  this->declare_parameter("default_cmd1", 0.0);
  this->declare_parameter("default_cmd2", 0.0);
  this->declare_parameter("default_cmd3", 0.0);
  this->declare_parameter("default_cmd4", 0.0);
}

void TuningSignalGenerator::update_params()
{
  // controller_mode
  controller_mode_ = static_cast<RosCopterControllerMode>(this->get_parameter("controller_mode").as_int());

  // Controller output
  controller_output_ = static_cast<ControllerOutput>(this->get_parameter("controller_output").as_int() - 1);

  // signal_type
  std::string signal_type_string = this->get_parameter("signal_type").as_string();
  if (signal_type_string == "step") {
    signal_type_ = SignalType::STEP;
  } else if (signal_type_string == "square") {
    signal_type_ = SignalType::SQUARE;
  } else if (signal_type_string == "sawtooth") {
    signal_type_ = SignalType::SAWTOOTH;
  } else if (signal_type_string == "triangle") {
    signal_type_ = SignalType::TRIANGLE;
  } else if (signal_type_string == "sine") {
    signal_type_ = SignalType::SINE;
  } else {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Param signal_type set to invalid type %s!",
                 signal_type_string.c_str());
  }

  // publish_rate_hz
  double publish_rate_hz_value = this->get_parameter("publish_rate_hz").as_double();
  if (publish_rate_hz_value <= 0) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Param publish_rate_hz must be greater than 0!");
  } else {
    // Parameter has changed, create new timer with updated value
    if (publish_rate_hz_ != publish_rate_hz_value) {
      publish_rate_hz_ = publish_rate_hz_value;
      publish_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<long>(1000 / publish_rate_hz_)),
        std::bind(&TuningSignalGenerator::publish_timer_callback, this));
    }
  }

  // signal_magnitude_
  signal_magnitude_ = this->get_parameter("signal_magnitude").as_double();

  // frequency_hz
  double frequency_hz_value = this->get_parameter("frequency_hz").as_double();
  if (frequency_hz_value <= 0) {
    RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "Param frequency_hz must be greater than 0!");
  } else {
    frequency_hz_ = frequency_hz_value;
  }


  default_cmd1_ = this->get_parameter("default_cmd1").as_double();
  default_cmd2_ = this->get_parameter("default_cmd2").as_double();
  default_cmd3_ = this->get_parameter("default_cmd3").as_double();
  default_cmd4_ = this->get_parameter("default_cmd4").as_double();
}

void TuningSignalGenerator::reset()
{
  initial_time_ = this->get_clock()->now().seconds();
  paused_time_ = 0;
  is_paused_ = true;
  single_period_start_time_ = 0;
  step_toggled_ = false;
}

} // namespace roscopter

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roscopter::TuningSignalGenerator>());
  rclcpp::shutdown();
  return 0;
}
