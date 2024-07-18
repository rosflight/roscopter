#ifndef CONTROLLER_SUCCESSIVE_LOOP_H
#define CONTROLLER_SUCCESSIVE_LOOP_H

#include <controller/controller_ros.hpp>
#include <controller/simple_pid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <roscopter_msgs/msg/command.hpp>
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
  double throttle_cmd_;
  double phi_cmd_;
  double theta_cmd_;
  double tau_x_;
  double tau_y_;
  double tau_z_;

  // PID Inner loops
  controller::SimplePID PID_phi_;
  controller::SimplePID PID_theta_;
  controller::SimplePID PID_psi_;
  controller::SimplePID PID_u_n_;
  controller::SimplePID PID_u_e_;
  controller::SimplePID PID_u_d_;

  // Functions
  rosflight_msgs::msg::Command compute_control(roscopter_msgs::msg::State xhat, roscopter_msgs::msg::Command input_cmd, double dt);
  void reset_integrators();
  void update_gains();
  void attitude_control(double phi_cmd, double theta_cmd, double psi_cmd, double dt, roscopter_msgs::msg::State xhat);
  void trajectory_control(double pn_cmd, double pe_cmd, double pd_cmd, double dt, roscopter_msgs::msg::State xhat);
  void declare_params();

};

}   // namespace controller

#endif
