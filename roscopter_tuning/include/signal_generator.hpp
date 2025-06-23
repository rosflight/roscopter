/*
 * Software License Agreement (BSD-3 License)
 *
 * Copyright (c) 2023 Brandon Sutherland, BYU MAGICC Lab.
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
 * ROS2 node for generating various input signals for tuning any/all layers of the controller.
 *
 * @author Brandon Sutherland <brandonsutherland2@gmail.com>
 */

#ifndef TUNING_SIGNAL_GENERATOR_HPP
#define TUNING_SIGNAL_GENERATOR_HPP

#include "roscopter_msgs/msg/controller_command.hpp"
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <unordered_set>
#include <system_error>
#include <sstream>

namespace roscopter
{
/**
 * This class is used to generate various input signals to test and tune all the control layers
 * in ROScopter. It currently supports square, sawtooth, triangle, and sine signals.
 */
class TuningSignalGenerator : public rclcpp::Node
{
public:
  /// Contructor for signal generator.
  TuningSignalGenerator();

private:
  /// This defines what level of the controller to publish the generated signal to.
  enum class RosCopterControllerMode
  {
    NPOS_EPOS_DPOS_YAW,
    NVEL_EVEL_DPOS_YAWRATE,
    NACC_EACC_DACC_YAWRATE,
    NVEL_EVEL_DVEL_YAWRATE,
    NPOS_EPOS_DVEL_YAW,
    ROLL_PITCH_YAW_THROTTLE,
    ROLL_PITCH_YAWRATE_THROTTLE,
    ROLLRATE_PITCHRATE_YAWRATE_THROTTLE,
    PASS_THROUGH_TO_MIXER,
    ROLL_PITCH_YAW_THRUST_TO_MIXER,
    ROLLRATE_PITCHRATE_YAWRATE_THRUST_TO_MIXER
  };

  /// Defines which channel (1-4) the controller will publish to
  enum class ControllerOutput
  {
    CMD1,
    CMD2,
    CMD3,
    CMD4
  };

  /// This defines what type of signal to publish to the selected controller.
  enum class SignalType
  {
    STEP,
    SQUARE,
    SAWTOOTH,
    TRIANGLE,
    SINE
  };
  std::unordered_set<std::string> signal_types_ = {"step", "square", "sawtooth", "triangle", "sine"};

  // Parameters
  RosCopterControllerMode controller_mode_; ///< Controller mode to output command signals to.
  ControllerOutput controller_output_; ///< Which channel the controller outputs to
  SignalType signal_type_;             ///< Signal type to output.
  double publish_rate_hz_;             ///< Frequency to publish commands.
  double signal_magnitude_;            ///< The the magnitude of the signal being generated.
  double frequency_hz_;                ///< Frequency of the signal.
  double default_cmd1_;               ///< Default cmd1 value.
  double default_cmd2_;               ///< Default cmd2 value.
  double default_cmd3_;               ///< Default cmd3 value.
  double default_cmd4_;               ///< Default cmd4 value.

  // Internal values
  bool step_toggled_;               ///< Flag for when step signal has been toggled.
  double initial_time_;             ///< Initial time of the signal.
  bool is_paused_;                  ///< Flag to specify if signal should be paused.
  double paused_time_;              ///< Amount of time that has been spent paused.
  double single_period_start_time_; ///< Epoch time of when single period start was called.

  /// Controller command ROS message publisher.
  rclcpp::Publisher<roscopter_msgs::msg::ControllerCommand>::SharedPtr command_publisher_;

  /// ROS timer to run timer callback, which publishes commands
  rclcpp::TimerBase::SharedPtr publish_timer_;

  /// ROS parameter change callback handler.
  OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  /// ROS service for toggling step signal.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr step_toggle_service_;
  /// ROS service for reset signal.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  /// ROS service for pause signal.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr pause_service_;
  /// ROS service for start signal continuously.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_continuous_service_;
  /// ROS service for start signal for one period.
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_single_service_;

  /// Callback to publish command on topic.
  void publish_timer_callback();

  /// Callback for parameter changes.
  rcl_interfaces::msg::SetParametersResult
  param_callback(const std::vector<rclcpp::Parameter> & params);

  /// Callback to toggle step signal.
  bool step_toggle_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                    const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /// Callback to reset signal.
  bool reset_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                              const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /// Callback to pause signal.
  bool pause_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                              const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /// Callback to start signal continuously.
  bool start_continuous_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                         const std_srvs::srv::Trigger::Response::SharedPtr & res);
  /// Callback to start signal for a single period.
  bool start_single_service_callback(const std_srvs::srv::Trigger::Request::SharedPtr & req,
                                     const std_srvs::srv::Trigger::Response::SharedPtr & res);

  /**
   * @brief Get the value for a step signal at the given time with the given conditions.
   *
   * @param step_toggled Flag to specify if signal is "stepped up" or not.
   * @param amplitude The amplitude of the signal.
   * @param center_value The central value of the signal. Not the initial value of the signal,
   *  but the value directly in the middle of the step values.
   */
  static double get_step_signal(bool step_toggled, double amplitude, double center_value);

  /**
   * @brief Get the value for a square signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param center_value The central value of the signal. The in other words, the signal 'offset'.
   */
  static double get_square_signal(double elapsed_time, double amplitude, double frequency,
                                  double center_value);
  /**
   * @brief Get the value for a sawtooth signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param center_value The central value of the signal. The in other words, the signal 'offset'.
   */
  static double get_sawtooth_signal(double elapsed_time, double amplitude, double frequency,
                                    double center_value);
  /**
   * @brief Get the value for a triangle signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param center_value The central value of the signal. The in other words, the signal 'offset'.
   */
  static double get_triangle_signal(double elapsed_time, double amplitude, double frequency,
                                    double center_value);
  /**
   * @brief Get the value for a sine signal at the given time with the given conditions.
   *
   * @param elapsed_time The amount of time that has passed since the 'start' of the signal
   *   in seconds.
   * @param amplitude The amplitude of the signal.
   * @param frequency The frequency of the signal.
   * @param center_value The central value of the signal. The in other words, the signal 'offset'.
   */
  static double get_sine_signal(double elapsed_time, double amplitude, double frequency,
                                double center_value);

  /// Declares parameters and the parameter descriptors with ROS2
  void declare_params();
  /// Updates the parameters within the class with the latest values from ROS.
  void update_params();

  /// Reset the signal generator.
  void reset();
};
} // namespace roscopter

#endif // TUNING_SIGNAL_GENERATOR_HPP
