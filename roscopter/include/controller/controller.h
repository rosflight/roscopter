#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/status.hpp>
#include <controller/simple_pid.h>
#include <nav_msgs/msg/odometry.hpp>
#include <roscopter_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <stdint.h>
// #include <dynamic_reconfigure/server.h>
// #include <roscopter/ControllerConfig.h>

using std::placeholders::_1;

namespace controller
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

class Controller : public rclcpp::Node
{

public:

  Controller();

private:

  // Publishers and Subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr state_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::Bool>::SharedPtr is_flying_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::Command>::SharedPtr cmd_sub_;
  rclcpp::Subscription<rosflight_msgs::msg::Status>::SharedPtr status_sub_;

  rclcpp::Publisher<rosflight_msgs::msg::Command>::SharedPtr command_pub_;

  // Paramters
  double throttle_eq_;
  double mass_;
  double max_thrust_;
  double max_accel_xy_;
  double max_accel_z_;
  double min_altitude_;
  bool is_flying_;
  bool armed_;
  bool received_cmd_;

  // PID Controllers
  controller::SimplePID PID_x_dot_;
  controller::SimplePID PID_y_dot_;
  controller::SimplePID PID_z_dot_;
  controller::SimplePID PID_n_;
  controller::SimplePID PID_e_;
  controller::SimplePID PID_d_;
  controller::SimplePID PID_psi_;

  // Memory for sharing information between functions
  state_t xhat_ = {}; // estimate
  max_t max_ = {};
  rosflight_msgs::msg::Command command_;
  command_t xc_ = {}; // command
  double prev_time_;
  uint8_t control_mode_;

  // Functions
  void stateCallback(const nav_msgs::msg::Odometry &msg);
  void isFlyingCallback(const roscopter_msgs::msg::Bool &msg);
  void cmdCallback(const roscopter_msgs::msg::Command &msg);
  void statusCallback(const rosflight_msgs::msg::Status &msg);

  void computeControl(double dt);
  void resetIntegrators();
  void publishCommand();
  double saturate(double x, double max, double min);

  /**
   * @brief Declares parameters with ROS2 and loads from a parameter file, if given
   */
  void declareParams();

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  /**
   * @brief Callback function to handle parameter changes
   * 
   * @param parameters: Vector of rclcpp::Parameter objects that were changed
   */
  rcl_interfaces::msg::SetParametersResult parametersCallback(const std::vector<rclcpp::Parameter> & parameters);

};
}

#endif
