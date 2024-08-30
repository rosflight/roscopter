#ifndef CONTROLLER_ROS_H
#define CONTROLLER_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/controller_command.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <roscopter_msgs/msg/bool.hpp>
#include <param_manager/param_manager.hpp>
#include <stdint.h>

using std::placeholders::_1;

namespace roscopter
{


class ControllerROS : public rclcpp::Node
{

public:

  ControllerROS();

  static double saturate(double x, double max, double min);

protected:
  ParamManager params;  
  roscopter_msgs::msg::State xhat_;

private:

  // Publishers and Subscribers
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::ControllerCommand>::SharedPtr cmd_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr command_pub_;

  // Memory for sharing information between functions
  roscopter_msgs::msg::ControllerCommand input_cmd_;  /** High level, input control commands to the autopilot */
  rosflight_msgs::msg::Status status_;      /** Contains information about whether or not the vehicle is armed */

  // Functions
  double compute_dt(double now);
  void state_callback(const roscopter_msgs::msg::State &msg);
  void cmd_callback(const roscopter_msgs::msg::ControllerCommand &msg);
  void status_callback(const rosflight_msgs::msg::Status &msg);
  void publish_command(rosflight_msgs::msg::Command &command);

  virtual rosflight_msgs::msg::Command manage_state(roscopter_msgs::msg::ControllerCommand & input_cmd, rosflight_msgs::msg::Status & status_msg, double dt) = 0;
  virtual void update_gains() = 0;

  /**
   * @brief Declares parameters with ROS2 and loads from a parameter file, if given
   */
  void declare_params();

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  /**
   * @brief Callback function to handle parameter changes
   * 
   * @param parameters: Vector of rclcpp::Parameter objects that were changed
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

};
}

#endif
