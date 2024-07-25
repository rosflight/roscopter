#ifndef CONTROLLER_CASCADING_PID_HPP
#define CONTROLLER_CASCADING_PID_HPP

#include <controller/controller_ros.hpp>
#include <controller/controller_state_machine.hpp>
#include <controller/simple_pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/controller_command.hpp>
#include <roscopter_msgs/msg/controller_internals.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <rosflight_msgs/msg/command.hpp>

using std::placeholders::_1;

namespace roscopter
{

class ControllerCascadingPID : public ControllerStateMachine
{

public:
  ControllerCascadingPID();

private:
  rclcpp::Publisher<roscopter_msgs::msg::ControllerInternals>::SharedPtr controller_internals_pub_;

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

  // PID Controllers
  roscopter::SimplePID PID_phi_;
  roscopter::SimplePID PID_theta_;
  roscopter::SimplePID PID_psi_;
  roscopter::SimplePID PID_u_n_;
  roscopter::SimplePID PID_u_e_;
  roscopter::SimplePID PID_u_d_;
  roscopter::SimplePID PID_x_dot_;
  roscopter::SimplePID PID_y_dot_;
  roscopter::SimplePID PID_z_dot_;
  roscopter::SimplePID PID_n_;
  roscopter::SimplePID PID_e_;
  roscopter::SimplePID PID_d_;

  // Functions
  rosflight_msgs::msg::Command compute_offboard_control(roscopter_msgs::msg::ControllerCommand & input_cmd, double dt);
  void reset_integrators();
  void update_gains() override;

  void declare_params();

  // Control helper functions
  void npos_epos_dpos_yaw(roscopter_msgs::msg::ControllerCommand input_cmd);
  void nvel_evel_dvel_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd);
  void nacc_eacc_dacc_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd);
  void npos_epos_dvel_yaw(roscopter_msgs::msg::ControllerCommand input_cmd);
  void nvel_evel_dpos_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd);
  void roll_pitch_yaw_throttle_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd);
  void roll_pitch_yaw_throttle(roscopter_msgs::msg::ControllerCommand input_cmd);
  void pass_to_firmware_controller(roscopter_msgs::msg::ControllerCommand input_cmd);
};

}   // namespace controller

#endif
