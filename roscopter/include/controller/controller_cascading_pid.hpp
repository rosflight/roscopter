#ifndef CONTROLLER_CASCADING_PID_HPP
#define CONTROLLER_CASCADING_PID_HPP

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
  roscopter::SimplePID PID_roll_;
  roscopter::SimplePID PID_pitch_;
  roscopter::SimplePID PID_yaw_;

  // PID controller from rates to torque
  roscopter::SimplePID PID_roll_rate_;
  roscopter::SimplePID PID_pitch_rate_;
  roscopter::SimplePID PID_yaw_rate_;

  // PID controller from velocities to accelerations
  roscopter::SimplePID PID_vel_n_;
  roscopter::SimplePID PID_vel_e_;
  roscopter::SimplePID PID_vel_d_;

  // PID controller from positions to velocities
  roscopter::SimplePID PID_n_;
  roscopter::SimplePID PID_e_;
  roscopter::SimplePID PID_d_;

  // PID controller from yaw to yaw rate
  roscopter::SimplePID PID_yaw_to_rate_;

  // Functions
  rosflight_msgs::msg::Command compute_offboard_control(roscopter_msgs::msg::ControllerCommand & input_cmd, double dt);
  void reset_integrators();
  void update_gains() override;
  double calculate_max_xy_accel(double max_accel_z, double equilibrium_throttle);

  void declare_params();

  // Control helper functions
  void npos_epos_dpos_yaw(roscopter_msgs::msg::ControllerCommand input_cmd);
  void nvel_evel_dvel_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd);
  void nacc_eacc_dacc_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd);
  void npos_epos_dvel_yaw(roscopter_msgs::msg::ControllerCommand input_cmd);
  void nvel_evel_dpos_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd);
  void roll_pitch_yaw_throttle(roscopter_msgs::msg::ControllerCommand input_cmd);
  void pass_to_firmware_controller(roscopter_msgs::msg::ControllerCommand input_cmd);
  void roll_pitch_yaw_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd);
  void rollrate_pitchrate_yawrate_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd);
};

}   // namespace controller

#endif
