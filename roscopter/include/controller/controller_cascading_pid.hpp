#ifndef CONTROLLER_CASCADING_PID_HPP
#define CONTROLLER_CASCADING_PID_HPP

#include <cfloat>
#include <Eigen/Geometry>

#include <controller/controller_state_machine.hpp>
#include <controller/simple_pid.hpp>

#include <roscopter_msgs/msg/controller_command.hpp>
#include <rosflight_msgs/msg/command.hpp>

using std::placeholders::_1;

#define TO_RADIANS M_PI/180.0

namespace roscopter
{

class ControllerCascadingPID : public ControllerStateMachine
{

public:
  ControllerCascadingPID();

private:
  rosflight_msgs::msg::Command output_cmd_;
  double dt_;
  bool params_initialized_;

  // PID controller from angle to torque
  roscopter::SimplePID PID_roll_to_torque_;
  roscopter::SimplePID PID_pitch_to_torque_;
  roscopter::SimplePID PID_yaw_to_torque_;

  // PID controller from rates to torque
  roscopter::SimplePID PID_roll_rate_to_torque_;
  roscopter::SimplePID PID_pitch_rate_to_torque_;
  roscopter::SimplePID PID_yaw_rate_to_torque_;

  // PID controller from velocities to accelerations
  roscopter::SimplePID PID_vel_n_to_accel_;
  roscopter::SimplePID PID_vel_e_to_accel_;
  roscopter::SimplePID PID_vel_d_to_accel_;

  // PID controller from positions to velocities
  roscopter::SimplePID PID_n_to_vel_;
  roscopter::SimplePID PID_e_to_vel_;
  roscopter::SimplePID PID_d_to_vel_;

  // PID controller from yaw to yaw rate
  roscopter::SimplePID PID_yaw_to_rate_;

  // Functions
  rosflight_msgs::msg::Command compute_offboard_control(roscopter_msgs::msg::ControllerCommand & input_cmd, double dt);
  void reset_integrators();
  void update_gains() override;
  // double calculate_max_xy_accel(double max_accel_z, double equilibrium_throttle);

  void declare_params();

  // Control helper functions
  void npos_epos_dpos_yaw(roscopter_msgs::msg::ControllerCommand input_cmd);
  void nvel_evel_dvel_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd);
  void facc_racc_dacc_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd);
  void npos_epos_dvel_yaw(roscopter_msgs::msg::ControllerCommand input_cmd);
  void nvel_evel_dpos_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd);
  void roll_pitch_yaw_throttle(roscopter_msgs::msg::ControllerCommand input_cmd);
  void pass_to_firmware_controller(roscopter_msgs::msg::ControllerCommand input_cmd);
  void roll_pitch_yaw_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd);
  void roll_pitch_yawrate_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd);
  void rollrate_pitchrate_yawrate_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd);
};

}   // namespace controller

#endif
