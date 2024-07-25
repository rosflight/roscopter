#include "navigation/trajectory_follower.hpp"

namespace roscopter
{
  
TrajectoryFollower::TrajectoryFollower() : TrajectoryFollowerROS()
{
  declare_params();
  params.set_parameters();
}

void TrajectoryFollower::declare_params()
{
  params.declare_double("mass", 2.0);
  params.declare_double("gravity", 9.81);
  params.declare_double("max_roll", 30.0);
  params.declare_double("max_pitch", 30.0);

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

  params.declare_double("tau", 0.05);
}

void TrajectoryFollower::update_gains()
{
  double P, I, D, tau;
  tau = params.get_double("tau");
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

roscopter_msgs::msg::ControllerCommand TrajectoryFollower::manage_trajectory(roscopter_msgs::msg::TrajectoryCommand input_cmd, double dt)
{
  double mass = params.get_double("mass");
  double g = params.get_double("gravity");
  double max_roll = params.get_double("max_roll") * M_PI / 180.0;
  double max_pitch = params.get_double("max_pitch") * M_PI / 180.0;
  double equilibrium_throttle = params.get_double("equilibrium_throttle");
  double max_throttle = params.get_double("max_throttle");
  double min_throttle = params.get_double("min_throttle");

  dt_ = dt;
  if (dt_ < 0.0000001) {
    // PID loops can blow up if dt is too small.
    return output_cmd_;
  }

  double u_n = north_control(input_cmd.position[0], input_cmd.velocity[0], input_cmd.acceleration[0]);
  double u_e = east_control(input_cmd.position[1], input_cmd.velocity[1], input_cmd.acceleration[1]);
  double u_d = down_control(input_cmd.position[2], input_cmd.velocity[2], input_cmd.acceleration[2]);

  // Compute command euler angles and thrust PID control outputs
  // Make sure to saturate the phi and theta commands 
  double phi_cmd_unsat = u_e*cos(xhat_.psi)/g - u_n*sin(xhat_.psi)/g;
  double theta_cmd_unsat = -u_n*cos(xhat_.psi)/g - u_e*sin(xhat_.psi)/g;
  double thrust_cmd = mass * (g - u_d);

  // Convert thrust command to throttle setting
  double throttle_setting = thrust_cmd * equilibrium_throttle / (mass * g);

  // Construct output
  output_cmd_.mode = roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAW_THROTTLE;
  output_cmd_.cmd1 = saturate(phi_cmd_unsat, max_roll, -max_roll);
  output_cmd_.cmd2 = saturate(theta_cmd_unsat, max_pitch, -max_pitch);
  output_cmd_.cmd3 = wrap_within_180(0.0, input_cmd.psi_cmd);
  output_cmd_.cmd4 = saturate(throttle_setting, max_throttle, min_throttle);

  return output_cmd_;
}

double TrajectoryFollower::wrap_within_180(double datum, double angle_to_wrap)
{
  RCLCPP_INFO_STREAM(this->get_logger(), "PSI_cmd: " << angle_to_wrap);
  while (fabs(datum - angle_to_wrap) > M_PI) {
    if (datum - angle_to_wrap > M_PI) {
      angle_to_wrap -= 2*M_PI;
    } else {
      angle_to_wrap += 2*M_PI;
    }
  }
  RCLCPP_INFO_STREAM(this->get_logger(), "PSI_cmd: " << angle_to_wrap);
  return angle_to_wrap;
}

double TrajectoryFollower::north_control(double pn_cmd, double pn_dot_cmd, double pn_ddot_cmd)
{
  double C_d = params.get_double("C_d");
  double g = params.get_double("gravity");

  // North control effort - Eq. 14.34. Note the negative velocity passed to the PID object
  double pn_dot_tilde = pn_dot_cmd - xhat_.v_n;;
  double u_n = pn_ddot_cmd + g*C_d*xhat_.v_n + PID_u_n_.compute_pid(pn_cmd, xhat_.position[0], dt_, -pn_dot_tilde);

  return u_n;
}

double TrajectoryFollower::east_control(double pe_cmd, double pe_dot_cmd, double pe_ddot_cmd)
{
  double C_d = params.get_double("C_d");
  double g = params.get_double("gravity");

  // East control effort - Eq. 14.34. Note the negative velocity passed to the PID object
  double pe_dot_tilde = pe_dot_cmd - xhat_.v_e;
  double u_e = pe_ddot_cmd + g*C_d*xhat_.v_e + PID_u_e_.compute_pid(pe_cmd, xhat_.position[1], dt_, -pe_dot_tilde);

  return u_e;
}

double TrajectoryFollower::down_control(double pd_cmd, double pd_dot_cmd, double pd_ddot_cmd)
{
  double C_d = params.get_double("C_d");
  double g = params.get_double("gravity");

  // Down control effort - Eq. 14.34. Note the negative velocity passed to the PID object
  double pd_dot_tilde = pd_dot_cmd - xhat_.v_d;
  double u_d = pd_ddot_cmd + g*C_d*xhat_.v_d + PID_u_d_.compute_pid(pd_cmd, xhat_.position[2], dt_, -pd_dot_tilde);

  return u_d;
}

} // namespace roscopter