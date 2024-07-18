#include <controller/controller_successive_loop.hpp>

using std::placeholders::_1;

namespace roscopter
{

ControllerSuccessiveLoop::ControllerSuccessiveLoop() : ControllerROS()
{
  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
//   double equilibrium_throttle = params.get_double("equilibrium_throttle");
//   max_accel_z_ = 1.0 / equilibrium_throttle;
//   max_accel_xy_ = sin(acos(equilibrium_throttle)) / equilibrium_throttle / sqrt(2.);

  // Define and declare parameters here that pertain to the ControllerSuccessiveLoop
  declare_params();
  params.set_parameters();

  update_gains();
  reset_integrators();
}

void ControllerSuccessiveLoop::declare_params() {
  params.declare_double("phi_kp", 0.2);
  params.declare_double("phi_ki", 0.0);
  params.declare_double("phi_kd", 0.2);
  params.declare_double("max_phi_dot", 5.0);

  params.declare_double("theta_kp", 0.2);
  params.declare_double("theta_ki", 0.0);
  params.declare_double("theta_kd", 0.2);
  params.declare_double("max_theta_dot", 5.0);

  params.declare_double("psi_kp", 0.2);
  params.declare_double("psi_ki", 0.002);
  params.declare_double("psi_kd", 0.2);
  params.declare_double("max_psi_dot", 5.0);

  params.declare_double("u_n_kp", 0.2);
  params.declare_double("u_n_ki", 0.002);
  params.declare_double("u_n_kd", 0.2);
  params.declare_double("max_u_n_dot", 5.0);

  params.declare_double("u_e_kp", 0.2);
  params.declare_double("u_e_ki", 0.002);
  params.declare_double("u_e_kd", 0.2);
  params.declare_double("max_u_e_dot", 5.0);

  params.declare_double("u_d_kp", 0.2);
  params.declare_double("u_d_ki", 0.002);
  params.declare_double("u_d_kd", 0.2);
  params.declare_double("max_u_d_dot", 5.0);

  params.declare_double("C_d", 0.0);
  params.declare_double("gravity", 9.81);
  params.declare_double("mass", 2.0);

  params.declare_double("min_throttle", 0.0);
  params.declare_double("max_roll_torque", 0.1);
  params.declare_double("max_pitch_torque", 0.0);
  params.declare_double("max_yaw_torque", 0.0);
}

void ControllerSuccessiveLoop::update_gains() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Updating gains!");
    double P, I, D, tau;
    tau = params.get_double("tau");
    P = params.get_double("phi_kp");
    I = params.get_double("phi_ki");
    D = params.get_double("phi_kd");
    double max_phi_dot = params.get_double("max_phi_dot");
    PID_phi_.set_gains(P, I, D, tau, max_phi_dot, -max_phi_dot);

    P = params.get_double("theta_kp");
    I = params.get_double("theta_ki");
    D = params.get_double("theta_kd");
    double max_theta_dot = params.get_double("max_theta_dot");
    PID_theta_.set_gains(P, I, D, tau, max_theta_dot, -max_theta_dot);

    P = params.get_double("psi_kp");
    I = params.get_double("psi_ki");
    D = params.get_double("psi_kd");
    double max_psi_dot = params.get_double("max_psi_dot");
    PID_psi_.set_gains(P, I, D, tau, max_psi_dot, -max_psi_dot);

    P = params.get_double("u_n_kp");
    I = params.get_double("u_n_ki");
    D = params.get_double("u_n_kd");
    double max_u_n_dot = params.get_double("max_u_n_dot");
    PID_u_n_.set_gains(P, I, D, tau, max_u_n_dot, -max_u_n_dot);

    P = params.get_double("u_e_kp");
    I = params.get_double("u_e_ki");
    D = params.get_double("u_e_kd");
    double max_u_e_dot = params.get_double("max_u_e_dot");
    PID_u_e_.set_gains(P, I, D, tau, max_u_e_dot, -max_u_e_dot);

    P = params.get_double("u_d_kp");
    I = params.get_double("u_d_ki");
    D = params.get_double("u_d_kd");
    double max_u_d_dot = params.get_double("max_u_d_dot");
    PID_u_d_.set_gains(P, I, D, tau, max_u_d_dot, -max_u_d_dot);
}

rosflight_msgs::msg::Command ControllerSuccessiveLoop::compute_control(roscopter_msgs::msg::State xhat, roscopter_msgs::msg::Command input_cmd, double dt)
{
  double max_throttle = params.get_double("max_throttle");
  double min_throttle = params.get_double("min_throttle");
  double max_roll_torque = params.get_double("max_roll_torque");
  double max_pitch_torque = params.get_double("max_pitch_torque");
  double max_yaw_torque = params.get_double("max_yaw_torque");
  
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
  double C_d = params.get_double("C_d");
  double g = params.get_double("gravity");
  double mass = params.get_double("mass");
  double equilibrium_throttle = params.get_double("equilibrium_throttle");

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
  double u_e = pe_ddot_cmd + g*C_d*pe_dot_cmd + PID_u_e_.compute_pid(pe_cmd, xhat.position[1], dt, pe_dot_tilde);

  // Down control effort
  double pd_dot_tilde = pd_dot_cmd - pd_dot_cmd;
  double u_d = pd_ddot_cmd + g*C_d*pd_dot_cmd + PID_u_d_.compute_pid(pd_cmd, xhat.position[2], dt, pd_dot_tilde);

  // Compute command euler angles
  phi_cmd_ = u_e*cos(xhat.psi)/g - u_n*sin(xhat.psi)/g;
  theta_cmd_ = -u_n*cos(xhat.psi)/g - u_e*sin(xhat.psi)/g;

  // Compute thrust and convert to throttle setting
  // Note that this is different than the commanded thrust calculation in Ch14 of the book, to accomodate equilibrium throttle setting without scaling the commanded calculated from the PID loop
  double thrust = - mass * u_d;

  throttle_cmd_ = equilibrium_throttle + thrust / (mass * g);
  RCLCPP_INFO_STREAM(this->get_logger(), "Thrust: " << thrust << " Throttle Setting: " << throttle_cmd_);
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

