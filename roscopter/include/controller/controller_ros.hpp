#ifndef CONTROLLER_ROS_H
#define CONTROLLER_ROS_H

#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <roscopter_msgs/msg/bool.hpp>
#include <param_manager/param_manager.hpp>
#include <stdint.h>

using std::placeholders::_1;

namespace roscopter
{

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double u;
  double v;
  double w;

  double p;
  double q;
  double r;
} state_t;

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double x_dot;
  double y_dot;
  double z_dot;

  double r;

  double ax;
  double ay;
  double az;

  double throttle;
} command_t;

typedef struct
{
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double n_dot;
  double e_dot;
  double d_dot;
} max_t;

class ControllerROS : public rclcpp::Node
{

public:

  ControllerROS();

  double saturate(double x, double max, double min);

protected:
  ParamManager params;  

private:

  // Publishers and Subscribers
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::Bool>::SharedPtr is_flying_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::Command>::SharedPtr cmd_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;
  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr command_pub_;

  // Memory for sharing information between functions
  roscopter_msgs::msg::Command input_cmd_;  /** High level, input control commands to the autopilot */
  roscopter_msgs::msg::Bool is_flying_;     /** Flag whether or not the vehicle is flying */
  rosflight_msgs::msg::Status status_;      /** Contains information about whether or not the vehicle is armed */
  bool received_cmd_;   /** Flag whether or not the controller received a high level command */
  bool first_active_control_loop_;

  // Functions
  void state_callback(const roscopter_msgs::msg::State &msg);
  void is_flying_callback(const roscopter_msgs::msg::Bool &msg);
  void cmd_callback(const roscopter_msgs::msg::Command &msg);
  void status_callback(const rosflight_msgs::msg::Status &msg);
  void publish_command(rosflight_msgs::msg::Command &command);

  virtual rosflight_msgs::msg::Command compute_control(roscopter_msgs::msg::State xhat, roscopter_msgs::msg::Command input_cmd, double dt) = 0;
  virtual void reset_integrators() = 0;
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
