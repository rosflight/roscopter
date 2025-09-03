#include "navigation/trajectory_follower.hpp"

namespace roscopter
{
  
TrajectoryFollower::TrajectoryFollower() : TrajectoryFollowerROS(), params_initialized_(false)
{
  declare_params();
  params.set_parameters();

  params_initialized_ = true;
  // Update gains after all the parameters have been set to initialize the PID controllers
  update_gains();
}

void TrajectoryFollower::declare_params()
{
  params.declare_double("mass", 2.0);
  params.declare_double("gravity", 9.81);
  params.declare_double("tau", 0.05);
  params.declare_double("C_d", 0.0);

  params.declare_double("u_n_kp", 0.2);
  params.declare_double("u_n_ki", 0.002);
  params.declare_double("u_n_kd", 0.2);

  params.declare_double("u_e_kp", 0.2);
  params.declare_double("u_e_ki", 0.002);
  params.declare_double("u_e_kd", 0.2);

  params.declare_double("u_d_kp", 0.2);
  params.declare_double("u_d_ki", 0.002);
  params.declare_double("u_d_kd", 0.2);

  params.declare_double("yaw_to_rate_kp", 0.2);
  params.declare_double("yaw_to_rate_ki", 0.002);
  params.declare_double("yaw_to_rate_kd", 0.2);

  params.declare_double("equilibrium_throttle", 0.5);
  params.declare_double("max_throttle", 0.85);
  params.declare_double("min_throttle", 0.3);
  params.declare_double("max_roll", 30.0);
  params.declare_double("max_pitch", 30.0);
  params.declare_double("max_yaw_rate", 1.0);
  params.declare_double("max_descend_accel", 1.0);
  params.declare_double("down_command_window", 3.0);
}

void TrajectoryFollower::update_gains()
{
  // Make sure node is fully constructed before updating parameters
  if (!params_initialized_) { return; }

  double P, I, D, tau;

  // Calculate max accelerations. 
  double equilibrium_throttle = params.get_double("equilibrium_throttle");
  double g = params.get_double("gravity");
  max_accel_xy_ = sin(acos(equilibrium_throttle)) * g
        / equilibrium_throttle; // This assumes that the minimum vehicle-1 frame z acceleration is 1g
  max_accel_z_ = 1.0 / equilibrium_throttle * g;

  tau = params.get_double("tau");
  P = params.get_double("u_n_kp");
  I = params.get_double("u_n_ki");
  D = params.get_double("u_n_kd");
  PID_u_n_.set_gains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = params.get_double("u_e_kp");
  I = params.get_double("u_e_ki");
  D = params.get_double("u_e_kd");
  PID_u_e_.set_gains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = params.get_double("u_d_kp");
  I = params.get_double("u_d_ki");
  D = params.get_double("u_d_kd");
  double min = params.get_double("max_descend_accel") * g;
  PID_u_d_.set_gains(P, I, D, tau, min, -max_accel_z_);

  P = params.get_double("yaw_to_rate_kp");
  I = params.get_double("yaw_to_rate_ki");
  D = params.get_double("yaw_to_rate_kd");
  double max = params.get_double("max_yaw_rate");
  PID_yaw_to_rate_.set_gains(P, I, D, tau, max, -max);
}

roscopter_msgs::msg::ControllerCommand TrajectoryFollower::manage_trajectory(roscopter_msgs::msg::TrajectoryCommand input_cmd, double dt)
{
  // If RC has control, clear the integrators
  if (firmware_status_.rc_override) {
    clear_integrators();
  }

  double mass = params.get_double("mass");
  double g = params.get_double("gravity");
  double max_roll = params.get_double("max_roll") * M_PI / 180.0;
  double max_pitch = params.get_double("max_pitch") * M_PI / 180.0;

  dt_ = dt;
  if (dt_ < 0.0000001) {
    // PID loops can blow up if dt is too small.
    output_cmd_.cmd_valid = false;
    return output_cmd_;
  }

  // Rotate the velocity from the body frame to the inertial frame
  Eigen::Quaterniond q_body_to_inertial;
  if (xhat_.quat_valid) {
    q_body_to_inertial = Eigen::Quaterniond(xhat_.quat[0], xhat_.quat[1], xhat_.quat[2], xhat_.quat[3]);
  } else {
    double psi2 = xhat_.psi/2;
    double theta2 = xhat_.theta/2;
    double phi2 = xhat_.phi/2;
    double qw = cosf(psi2)*cosf(theta2)*cosf(phi2) + sinf(psi2)*sinf(theta2)*sinf(phi2);
    double qx = cosf(psi2)*cosf(theta2)*sinf(phi2) - sinf(psi2)*sinf(theta2)*cosf(phi2);
    double qy = cosf(psi2)*sinf(theta2)*cosf(phi2) + sinf(psi2)*cosf(theta2)*sinf(phi2);
    double qz = sinf(psi2)*cosf(theta2)*cosf(phi2) - cosf(psi2)*sinf(theta2)*sinf(phi2);
    q_body_to_inertial = Eigen::Quaterniond(qw, qx, qy, qz);
  }
  Eigen::Vector3d body_vels(xhat_.v_n, xhat_.v_e, xhat_.v_d);
  Eigen::Vector3d inertial_vels = q_body_to_inertial * body_vels;

  // Compute control from controller
  Eigen::Vector4d u_tilde = compute_control_input(input_cmd.position[0],
                                                  input_cmd.position[1],
                                                  input_cmd.position[2],
                                                  input_cmd.psi,
                                                  inertial_vels[0],
                                                  inertial_vels[1],
                                                  inertial_vels[2]);

  // Add feedforward commands to get total control vector u
  Eigen::Vector4d u_r;
  u_r << input_cmd.acceleration[0], input_cmd.acceleration[1], input_cmd.acceleration[2] - g, input_cmd.psi_dot;
  Eigen::Vector4d u = u_tilde + u_r;

  // Pass control through f^{-1} to get the angle commands
  Eigen::Vector4d nu = invert_control_inputs(u, mass, input_cmd.psi, input_cmd.psi_dot);

  // Unpack commands
  // Make sure to saturate the phi and theta commands 
  double thrust_cmd = nu[0];
  double phi_cmd_unsat = nu[1];
  double theta_cmd_unsat = nu[2];
  double r_cmd_unsat = nu[3];

  // Construct output
  double max_throttle = this->get_parameter("max_throttle").as_double();
  double min_throttle = this->get_parameter("min_throttle").as_double();
  double max_yawrate = this->get_parameter("max_yaw_rate").as_double();
  double equilibrium_throttle = this->get_parameter("equilibrium_throttle").as_double();
  output_cmd_.mode = roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAW_THRUST_TO_MIXER;
  output_cmd_.cmd1 = saturate(phi_cmd_unsat, max_roll, -max_roll);
  output_cmd_.cmd2 = saturate(theta_cmd_unsat, max_pitch, -max_pitch);
  output_cmd_.cmd3 = input_cmd.psi; //saturate(r_cmd_unsat, max_yawrate, -max_yawrate);
  // double throttle_cmd = equilibrium_throttle - thrust_cmd / g;
  // double throttle_cmd = thrust_cmd / (mass * g) * equilibrium_throttle;
  // output_cmd_.cmd4 = saturate(throttle_cmd, max_throttle, min_throttle);
  output_cmd_.cmd4 = saturate(thrust_cmd, max_throttle * mass * g / equilibrium_throttle, min_throttle * mass * g / equilibrium_throttle);
  // RCLCPP_INFO_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 500, "u_r:" << u_r.transpose() << "\nu:" << u.transpose() << "\nnu: " << nu.transpose());

  output_cmd_.cmd_valid = true;

  return output_cmd_;
}

Eigen::Vector4d TrajectoryFollower::invert_control_inputs(const Eigen::Vector4d u, const double mass, const double psi, const double psi_dot)
{
  // Invert to find thrust
  double thrust_cmd = mass * u.segment(0, 3).norm();

  // Invert to find theta and phi
  Eigen::Vector3d z = -1 * R_psi(psi) * u.segment(0, 3) * mass / thrust_cmd;
  double phi = asin(-1 * z[1]);
  double theta = atan2(z[0], z[2]);

  // Solve for theta_dot and compute r
  double theta_dot = compute_theta_dot(z, mass, thrust_cmd, psi, psi_dot, u);
  double r = psi_dot * cos(theta) * cos(phi) - theta_dot * sin(phi);

  // Return the new command vector
  Eigen::Vector4d nu(thrust_cmd, phi, theta, r);
  return nu;
}

double TrajectoryFollower::compute_theta_dot(const Eigen::Vector3d z, const double mass, double thrust, const double psi, const double psi_dot, const Eigen::Vector4d u)
{
  // This assumes that the accelerations are not a function of time... For small dt this is
  // reasonable accurate.
  double z1_dot = -mass / thrust * (-u[0]*sin(psi)*psi_dot + u[1]*cos(psi)*psi_dot);
  double z3_dot = 0.0;
  double theta_dot = 1 / (1 + pow(z[0] / z[2], 2)) * (z[2] * z1_dot - z[0]*z3_dot) / pow(z[2], 2);
  return theta_dot;
}

Eigen::Matrix3d TrajectoryFollower::R_psi(double psi)
{
  Eigen::Matrix3d R;
  R << cos(psi), sin(psi), 0, -sin(psi), cos(psi), 0, 0, 0, 1.0;
  return R;
}

double TrajectoryFollower::wrap_within_180(double datum, double angle_to_wrap)
{
  while (fabs(datum - angle_to_wrap) > M_PI) {
    if (datum - angle_to_wrap > M_PI) {
      angle_to_wrap -= 2*M_PI;
    } else {
      angle_to_wrap += 2*M_PI;
    }
  }
  return angle_to_wrap;
}

Eigen::Vector4d TrajectoryFollower::compute_control_input(const double pn_cmd,
                                                          const double pe_cmd,
                                                          const double pd_cmd,
                                                          const double psi_cmd,
                                                          const double vn,
                                                          const double ve,
                                                          const double vd)
{
  // PID controller for now
  double u_n = north_control(pn_cmd, vn);
  double u_e = east_control(pe_cmd, ve);
  double u_d = down_control(pd_cmd, vd);
  double psi = psi_control(psi_cmd);

  return Eigen::Vector4d(u_n, u_e, u_d, psi);
}

double TrajectoryFollower::north_control(const double pn_cmd, const double vn)
{
  // North control effort
  double u_n_unsat = PID_u_n_.compute_pid(pn_cmd, xhat_.position[0], dt_, -vn);
  return saturate(u_n_unsat, max_accel_xy_, -max_accel_xy_);
}

double TrajectoryFollower::east_control(const double pe_cmd, const double ve)
{
  // East control effort
  double u_e_unsat = PID_u_e_.compute_pid(pe_cmd, xhat_.position[1], dt_, -ve);
  return saturate(u_e_unsat, max_accel_xy_, -max_accel_xy_);
}

double TrajectoryFollower::down_control(const double pd_cmd, const double vd)
{
  double g = params.get_double("gravity");
  double min_accel_z = params.get_double("max_descend_accel") * g;
  double max_d_cmd = params.get_double("down_command_window");

  // Saturate the down command (only when positive) to the maximum down command window.
  // This helps to ensure that the copter maintains a controllable down velocity when descending
  double actual_down_cmd = pd_cmd;
  if ((pd_cmd - xhat_.position[2]) > max_d_cmd) {
    actual_down_cmd = max_d_cmd + xhat_.position[2];
  }

  // Down control effort
  double u_d_unsat = PID_u_d_.compute_pid(actual_down_cmd, xhat_.position[2], dt_, vd);
  return saturate(u_d_unsat, min_accel_z, -max_accel_z_);
}

double TrajectoryFollower::psi_control(const double psi_cmd)
{
  // Psi control effort
  double psi_unsat = PID_yaw_to_rate_.compute_pid(psi_cmd, xhat_.psi, dt_);
  return psi_unsat;
}

void TrajectoryFollower::clear_integrators()
{
  PID_u_n_.clear_integrator();
  PID_u_e_.clear_integrator();
  PID_u_d_.clear_integrator();
}

} // namespace roscopter
