#ifndef CONTROLLER_STATE_MACHINE_HPP
#define CONTROLLER_STATE_MACHINE_HPP

#include <controller/controller_ros.hpp>
#include <controller/simple_pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/controller_command.hpp>
#include <roscopter_msgs/msg/state.hpp>
#include <rosflight_msgs/msg/command.hpp>
#include <rosflight_msgs/msg/status.hpp>

using std::placeholders::_1;

namespace roscopter
{

class ControllerStateMachine : public ControllerROS
{

enum
{
  DISARM,
  TAKEOFF,
  OFFBOARD,
  POSITION_HOLD,
  LANDING  
};

public:

  ControllerStateMachine();

private:

  // Paramters
  double min_altitude_;
  bool state_transition_;
  int state_;
  double takeoff_n_pos_;
  double takeoff_e_pos_;
  double takeoff_yaw_;
  double start_position_hold_time_;
  bool do_land_;

  // Functions
  void declare_params();

  rosflight_msgs::msg::Command manage_state(roscopter_msgs::msg::ControllerCommand & input_cmd, rosflight_msgs::msg::Status & status_msg, double dt) override;
  void manage_disarm(bool armed, bool cmd_valid);
  rosflight_msgs::msg::Command manage_takeoff(double dt);
  rosflight_msgs::msg::Command manage_position_hold(double dt);
  rosflight_msgs::msg::Command manage_landing();

  virtual void update_gains();
  virtual rosflight_msgs::msg::Command compute_offboard_control(roscopter_msgs::msg::ControllerCommand & input_cmd, double dt) = 0;
  virtual void reset_integrators() = 0;
};

}   // namespace controller

#endif
