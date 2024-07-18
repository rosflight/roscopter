#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <controller/controller_ros.hpp>
#include <controller/simple_pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/command.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <rosflight_msgs/msg/command.hpp>

using std::placeholders::_1;

namespace roscopter
{

class Controller : public ControllerROS
{

public:

  Controller();

private:

  // Paramters
  double mass_;
  double max_thrust_;
  double max_accel_xy_;
  double max_accel_z_;
  double min_altitude_;

  // PID Controllers
  controller::SimplePID PID_x_dot_;
  controller::SimplePID PID_y_dot_;
  controller::SimplePID PID_z_dot_;
  controller::SimplePID PID_n_;
  controller::SimplePID PID_e_;
  controller::SimplePID PID_d_;
  controller::SimplePID PID_psi_;

  // Memory for sharing information between functions
  max_t max_ = {};
  rosflight_msgs::msg::Command output_cmd_;

  // Functions
  rosflight_msgs::msg::Command compute_control(roscopter_msgs::msg::State xhat, roscopter_msgs::msg::Command input_cmd, double dt);
  void reset_integrators();
  void update_gains();

};

}   // namespace controller

#endif
