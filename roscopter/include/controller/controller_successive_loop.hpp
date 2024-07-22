#ifndef CONTROLLER_SUCCESSIVE_LOOP_H
#define CONTROLLER_SUCCESSIVE_LOOP_H

#include <controller/controller_ros.hpp>
#include <controller/simple_pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/controller_command.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <rosflight_msgs/msg/command.hpp>

using std::placeholders::_1;

namespace roscopter
{

class ControllerSuccessiveLoop : public ControllerROS
{

public:
  ControllerSuccessiveLoop();

private:
  rosflight_msgs::msg::Command output_cmd_;
  double phi_cmd_;
  double theta_cmd_;
  double psi_cmd_;
  double thrust_cmd_;
  double tau_x_;
  double tau_y_;
  double tau_z_;
  double throttle_cmd_;
  double dt_;

  // PID Inner loops
  controller::SimplePID PID_phi_;
  controller::SimplePID PID_theta_;
  controller::SimplePID PID_psi_;
  controller::SimplePID PID_u_n_;
  controller::SimplePID PID_u_e_;
  controller::SimplePID PID_u_d_;

  // Functions
  rosflight_msgs::msg::Command compute_control(roscopter_msgs::msg::ControllerCommand & input_cmd, double dt);
  void reset_integrators();
  void update_gains();

  void declare_params();

  void trajectory_control(double pn_cmd, double pe_cmd, double pd_cmd);
  void attitude_control();
  float north_control(double pn_cmd);
  float east_control(double pe_cmd);
  float down_control(double pd_cmd);
  void saturate_commands();

};

}   // namespace controller

#endif
