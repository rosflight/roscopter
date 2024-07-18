#include <controller/controller_successive_loop.hpp>

using std::placeholders::_1;

namespace roscopter
{

ControllerSuccessiveLoop::ControllerSuccessiveLoop() : ControllerROS()
{
  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
//   double equilibrium_throttle = this->get_parameter("equilibrium_throttle").as_double();
//   max_accel_z_ = 1.0 / equilibrium_throttle;
//   max_accel_xy_ = sin(acos(equilibrium_throttle)) / equilibrium_throttle / sqrt(2.);

  declare_params();
  set_gains();

  reset_integrators();
}

void ControllerSuccessiveLoop::declare_params() {
  this->declare_parameter("phi_kp", 0.2);
  this->declare_parameter("phi_ki", 0.0);
  this->declare_parameter("phi_kd", 0.2);
  this->declare_parameter("max_phi_dot", 5.0);

  this->declare_parameter("theta_kp", 0.2);
  this->declare_parameter("theta_ki", 0.0);
  this->declare_parameter("theta_kd", 0.2);
  this->declare_parameter("max_theta_dot", 5.0);

  this->declare_parameter("psi_kp", 0.2);
  this->declare_parameter("psi_ki", 0.002);
  this->declare_parameter("psi_kd", 0.2);
  this->declare_parameter("max_psi_dot", 5.0);

  this->declare_parameter("u_n_kp", 0.2);
  this->declare_parameter("u_n_ki", 0.002);
  this->declare_parameter("u_n_kd", 0.2);
  this->declare_parameter("max_u_n_dot", 5.0);

  this->declare_parameter("u_e_kp", 0.2);
  this->declare_parameter("u_e_ki", 0.002);
  this->declare_parameter("u_e_kd", 0.2);
  this->declare_parameter("max_u_e_dot", 5.0);

  this->declare_parameter("u_d_kp", 0.2);
  this->declare_parameter("u_d_ki", 0.002);
  this->declare_parameter("u_d_kd", 0.2);
  this->declare_parameter("max_u_d_dot", 5.0);

  this->declare_parameter("C_d", 0.0);
  this->declare_parameter("gravity", 9.81);
  this->declare_parameter("mass", 2.0);

  this->declare_parameter("min_throttle", 0.0);
  this->declare_parameter("max_roll_torque", 0.1);
  this->declare_parameter("max_pitch_torque", 0.0);
  this->declare_parameter("max_yaw_torque", 0.0);
  this->declare_parameter("max_thrust", 4.0);
}

void ControllerSuccessiveLoop::set_gains() {
    double P, I, D, tau;
    tau = this->get_parameter("tau").as_double();
    P = this->get_parameter("phi_kp").as_double();
    I = this->get_parameter("phi_ki").as_double();
    D = this->get_parameter("phi_kd").as_double();
    double max_phi_dot = this->get_parameter("max_phi_dot").as_double();
    PID_phi_.set_gains(P, I, D, tau, max_phi_dot, -max_phi_dot);

    P = this->get_parameter("theta_kp").as_double();
    I = this->get_parameter("theta_ki").as_double();
    D = this->get_parameter("theta_kd").as_double();
    double max_theta_dot = this->get_parameter("max_theta_dot").as_double();
    PID_theta_.set_gains(P, I, D, tau, max_theta_dot, -max_theta_dot);

    P = this->get_parameter("psi_kp").as_double();
    I = this->get_parameter("psi_ki").as_double();
    D = this->get_parameter("psi_kd").as_double();
    double max_psi_dot = this->get_parameter("max_psi_dot").as_double();
    PID_psi_.set_gains(P, I, D, tau, max_psi_dot, -max_psi_dot);

    P = this->get_parameter("u_n_kp").as_double();
    I = this->get_parameter("u_n_ki").as_double();
    D = this->get_parameter("u_n_kd").as_double();
    double max_u_n_dot = this->get_parameter("max_u_n_dot").as_double();
    PID_u_n_.set_gains(P, I, D, tau, max_u_n_dot, -max_u_n_dot);

    P = this->get_parameter("u_e_kp").as_double();
    I = this->get_parameter("u_e_ki").as_double();
    D = this->get_parameter("u_e_kd").as_double();
    double max_u_e_dot = this->get_parameter("max_u_e_dot").as_double();
    PID_u_e_.set_gains(P, I, D, tau, max_u_e_dot, -max_u_e_dot);

    P = this->get_parameter("u_d_kp").as_double();
    I = this->get_parameter("u_d_ki").as_double();
    D = this->get_parameter("u_d_kd").as_double();
    double max_u_d_dot = this->get_parameter("max_u_d_dot").as_double();
    PID_u_d_.set_gains(P, I, D, tau, max_u_d_dot, -max_u_d_dot);
}

rosflight_msgs::msg::Command ControllerSuccessiveLoop::compute_control(roscopter_msgs::msg::State xhat, roscopter_msgs::msg::Command input_cmd, double dt)
{
  double max_throttle = this->get_parameter("max_throttle").as_double();
  double min_throttle = this->get_parameter("min_throttle").as_double();
  double max_roll_torque = this->get_parameter("max_roll_torque").as_double();
  double max_pitch_torque = this->get_parameter("max_pitch_torque").as_double();
  double max_yaw_torque = this->get_parameter("max_yaw_torque").as_double();
  
  if(dt <= 0.0000001) {
    // This messes up the derivative calculation in the PID controllers
    RCLCPP_WARN_STREAM(this->get_logger(), "dt < 0.0000001");
    // TODO: Do we need to keep the last cmd and return it here?
    return output_cmd_;
  }

  double pn_cmd = input_cmd.cmd1;
  double pe_cmd = input_cmd.cmd2;
  double pd_cmd = input_cmd.cmd3;
  double psi_cmd = input_cmd.cmd4;

  // Run outer loop to generate thrust, phi, and theta commands
  trajectory_control(pn_cmd, pe_cmd, pd_cmd, dt, xhat);

  // Run inner, attitude loop
  attitude_control(phi_cmd_, theta_cmd_, psi_cmd, dt, xhat);

  // Populate the fields in the output message
  output_cmd_.mode = rosflight_msgs::msg::Command::MODE_PASS_THROUGH;
  output_cmd_.ignore = rosflight_msgs::msg::Command::IGNORE_NONE;

  output_cmd_.x = saturate(tau_x_, max_roll_torque, -max_roll_torque);
  output_cmd_.y = saturate(tau_y_, max_pitch_torque, -max_pitch_torque);
  output_cmd_.z = saturate(tau_z_, max_yaw_torque, -max_yaw_torque);
  output_cmd_.f = saturate(throttle_cmd_, max_throttle, min_throttle);

  return output_cmd_;
}

void ControllerSuccessiveLoop::attitude_control(double phi_cmd, double theta_cmd, double psi_cmd, double dt, roscopter_msgs::msg::State xhat)
{
    tau_x_ = PID_phi_.compute_pid(phi_cmd, xhat.phi, dt, xhat.p);
    tau_y_ = PID_theta_.compute_pid(theta_cmd, xhat.theta, dt, xhat.q);
    tau_z_ = PID_psi_.compute_pid(psi_cmd, xhat.psi, dt, xhat.r);
}

void ControllerSuccessiveLoop::trajectory_control(double pn_cmd, double pe_cmd, double pd_cmd, double dt, roscopter_msgs::msg::State xhat)
{
  double C_d = this->get_parameter("C_d").as_double();
  double g = this->get_parameter("gravity").as_double();
  double mass = this->get_parameter("mass").as_double();
  double max_thrust = this->get_parameter("max_thrust").as_double();

  // Optional feedforward commands
  double pn_ddot_cmd = 0.0;
  double pe_ddot_cmd = 0.0;
  double pd_ddot_cmd = 0.0;
  double pn_dot_cmd = 0.0;
  double pe_dot_cmd = 0.0;
  double pd_dot_cmd = 0.0;

  // North control effort
  double pn_dot_tilde = pn_dot_cmd - pn_dot_cmd;
  double u_n = pn_ddot_cmd + g*C_d*pn_dot_cmd + PID_u_n_.compute_pid(pn_cmd, xhat.position[0], dt, pn_dot_tilde);

  // East control effort
  double pe_dot_tilde = pe_dot_cmd - pe_dot_cmd;
  double u_e = pe_ddot_cmd + g*C_d*pe_dot_cmd + PID_u_n_.compute_pid(pe_cmd, xhat.position[1], dt, pe_dot_tilde);

  // Down control effort
  double pd_dot_tilde = pd_dot_cmd - pd_dot_cmd;
  double u_d = pd_ddot_cmd + g*C_d*pd_dot_cmd + PID_u_n_.compute_pid(pd_cmd, xhat.position[2], dt, pd_dot_tilde);

  // Compute command thrust and euler angles
  double thrust = mass * (g - u_d);
  phi_cmd_ = u_e*cos(xhat.psi)/g - u_n*sin(xhat.psi)/g;
  theta_cmd_ = -u_n*cos(xhat.psi)/g - u_e*sin(xhat.psi)/g;
  
  // Convert thrust to throttle setting
  throttle_cmd_ = thrust / max_thrust;
}

void ControllerSuccessiveLoop::reset_integrators()
{
    PID_phi_.clear_integrator();
    PID_theta_.clear_integrator();
    PID_psi_.clear_integrator();
    PID_u_n_.clear_integrator();
    PID_u_e_.clear_integrator();
    PID_u_d_.clear_integrator();
}

}  // namespace controller

