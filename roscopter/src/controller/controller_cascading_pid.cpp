#include <controller/controller_cascading_pid.hpp>

using std::placeholders::_1;

namespace roscopter
{

ControllerCascadingPID::ControllerCascadingPID() : ControllerStateMachine()
{
  // Declare and set parameters associated with Cascading PID controller
  declare_params();
  params.set_parameters();

  // Check to make sure equilibrium throttle is set properly
  if (!params.get_double("equilibrium_throttle")) {
    RCLCPP_ERROR(this->get_logger(), "Controller MAV equilibrium throttle not found!");
  }

  // Create the ROS publisher associated with this class
  controller_internals_pub_ = this->create_publisher<roscopter_msgs::msg::ControllerInternals>("controller_internals", 10);

  update_gains();
  reset_integrators();
}

void ControllerCascadingPID::declare_params() {
  params.declare_double("phi_kp", 0.2);
  params.declare_double("phi_ki", 0.0);
  params.declare_double("phi_kd", 0.2);

  params.declare_double("theta_kp", 0.2);
  params.declare_double("theta_ki", 0.0);
  params.declare_double("theta_kd", 0.2);

  params.declare_double("psi_kp", 0.2);
  params.declare_double("psi_ki", 0.002);
  params.declare_double("psi_kd", 0.2);

  params.declare_double("x_dot_P", 0.5);
  params.declare_double("x_dot_I", 0.0);
  params.declare_double("x_dot_D", 0.05);

  params.declare_double("y_dot_P", 0.5);
  params.declare_double("y_dot_I", 0.0);
  params.declare_double("y_dot_D", 0.05);

  params.declare_double("z_dot_P", 0.4);
  params.declare_double("z_dot_I", 0.25);
  params.declare_double("z_dot_D", 0.1);

  params.declare_double("north_P", 1.0);
  params.declare_double("north_I", 0.1);
  params.declare_double("north_D", 0.35);

  params.declare_double("east_P", 1.0);
  params.declare_double("east_I", 0.1);
  params.declare_double("east_D", 0.2);

  params.declare_double("down_P", 1.0);
  params.declare_double("down_I", 0.0);
  params.declare_double("down_D", 0.0);

  // System parameters
  params.declare_double("tau", 0.05);

  // Saturation Limit parameters
  params.declare_double("max_phi_dot", 5.0);
  params.declare_double("max_theta_dot", 5.0);
  params.declare_double("max_psi_dot", 5.0);
  params.declare_double("max_yaw_rate", 0.15);  // TODO: Can I get rid of this one for the above?
  params.declare_double("min_throttle", 0.0);
  params.declare_double("max_roll_torque", 1.0);
  params.declare_double("max_pitch_torque", 1.0);
  params.declare_double("max_yaw_torque", 1.0);
  params.declare_double("equilibrium_throttle", 0.5); 
  params.declare_double("max_roll", 0.30);  
  params.declare_double("max_pitch", 0.30);
  params.declare_double("max_throttle", 0.85);
  params.declare_double("max_n_dot", 0.15);
  params.declare_double("max_e_dot", 0.15);
  params.declare_double("max_d_dot", 0.15);

  params.declare_bool("pitch_tuning_override", false);
  params.declare_bool("roll_tuning_override", false);
  params.declare_double("min_attitude_altitude", 0.15);
}

void ControllerCascadingPID::update_gains() {
    RCLCPP_INFO_STREAM(this->get_logger(), "Updating gains!");

    // Update gains of this class
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

    // Calculate max accelerations. Assuming that equilibrium throttle produces
    // 1 g of acceleration and a linear thrust model, these max acceleration
    // values are computed in g's as well.
    double equilibrium_throttle = params.get_double("equilibrium_throttle");
    double max_accel_z = 1.0 / equilibrium_throttle;
    double max_accel_xy = sin(acos(equilibrium_throttle)) / equilibrium_throttle / sqrt(2.);

    P = params.get_double("x_dot_P");
    I = params.get_double("x_dot_I");
    D = params.get_double("x_dot_D");
    PID_x_dot_.set_gains(P, I, D, tau, max_accel_xy, -max_accel_xy);

    P = params.get_double("y_dot_P");
    I = params.get_double("y_dot_I");
    D = params.get_double("y_dot_D");
    PID_y_dot_.set_gains(P, I, D, tau, max_accel_xy, -max_accel_xy);

    P = params.get_double("z_dot_P");
    I = params.get_double("z_dot_I");
    D = params.get_double("z_dot_D");
    // set max z accelerations so that we can't fall faster than 1 gravity
    PID_z_dot_.set_gains(P, I, D, tau, 1.0, -max_accel_z);

    P = params.get_double("north_P");
    I = params.get_double("north_I");
    D = params.get_double("north_D");
    double max_n_dot = params.get_double("max_n_dot");
    PID_n_.set_gains(P, I, D, tau, max_n_dot, -max_n_dot);

    P = params.get_double("east_P");
    I = params.get_double("east_I");
    D = params.get_double("east_D");
    double max_e_dot = params.get_double("max_e_dot");
    PID_e_.set_gains(P, I, D, tau, max_e_dot, -max_e_dot);

    P = params.get_double("down_P");
    I = params.get_double("down_I");
    D = params.get_double("down_D");
    double max_d_dot = params.get_double("max_d_dot");
    PID_d_.set_gains(P, I, D, tau, max_d_dot, -max_d_dot);
}

rosflight_msgs::msg::Command ControllerCascadingPID::compute_offboard_control(roscopter_msgs::msg::ControllerCommand & input_cmd, double dt)
{
  // Note that dt is "safe", i.e., >0.0000001 as checked in the state machine
  dt_ = dt;

  uint8_t mode = input_cmd.mode;

  switch (mode) {
    case roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DPOS_YAW:
      npos_epos_dpos_yaw(input_cmd);
      break;
    
    case roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DPOS_YAWRATE:
      nvel_evel_dpos_yawrate(input_cmd);
      break;

    case roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE:
      nacc_eacc_dacc_yawrate(input_cmd);
      break;

    case roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE:
      nvel_evel_dvel_yawrate(input_cmd);
      break;

    case roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DVEL_YAW:
      npos_epos_dvel_yaw(input_cmd);
      break;
    
    // TODO: Change this as a flag in the message that checks if motor commands are desired
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAW_THROTTLE_TO_MOTOR:
      roll_pitch_yaw_throttle_to_motor(input_cmd);
      break;

    case roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAW_THROTTLE:
      roll_pitch_yaw_throttle(input_cmd);
      break;

    default:  // PASS_THROUGH, ROLLRATE_PITCHRATE_YAWRATE_THROTTLE, ROLL_PITCH_YAWRATE_THROTTLE
      pass_to_firmware_controller(input_cmd);
      break;
  }

  // // Check for tuning override flags before running the inner loop
  // if (params.get_bool("roll_tuning_override")) { phi_cmd_ = input_cmd.phi_c; }
  // if (params.get_bool("pitch_tuning_override")) { theta_cmd_ = input_cmd.theta_c; }

  // // Run inner, attitude loop
  // attitude_control();

  return output_cmd_;
}

void ControllerCascadingPID::roll_pitch_yaw_throttle_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // Update input command values (except throttle)
  input_cmd.cmd1 = PID_phi_.compute_pid(phi_cmd_, xhat_.phi, dt_, xhat_.p);
  input_cmd.cmd2 = PID_theta_.compute_pid(theta_cmd_, xhat_.theta, dt_, xhat_.q);
  input_cmd.cmd3 = PID_psi_.compute_pid(psi_cmd_, xhat_.psi, dt_, xhat_.r);
  
  // Update the mode with the correct type
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH;

  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::reset_integrators()
{
    PID_x_dot_.clear_integrator();
    PID_y_dot_.clear_integrator();
    PID_z_dot_.clear_integrator();
    PID_n_.clear_integrator();
    PID_e_.clear_integrator();
    PID_d_.clear_integrator();

    PID_phi_.clear_integrator();
    PID_theta_.clear_integrator();
    PID_psi_.clear_integrator();
    PID_u_n_.clear_integrator();
    PID_u_e_.clear_integrator();
    PID_u_d_.clear_integrator();
}


void ControllerCascadingPID::npos_epos_dpos_yaw(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double pn = input_cmd.cmd1;
  double pe = input_cmd.cmd2;
  double pd = input_cmd.cmd3;
  double psi = input_cmd.cmd4;

  // Figure out desired velocities (in inertial frame)
  // By running the position controllers
  double pndot_c = PID_n_.compute_pid(pn, xhat_.position[0], dt_);
  double pedot_c = PID_e_.compute_pid(pe, xhat_.position[1], dt_);
  double pddot_c = PID_d_.compute_pid(pd, xhat_.position[2], dt_);

  // Calculate desired yaw rate
  // TOOD: Wrap to within 180 deg
  // First, determine the shortest direction to the commanded psi
  if(fabs(psi + 2*M_PI - xhat_.psi) < fabs(psi - xhat_.psi))
  {
    psi += 2*M_PI;
  }
  else if (fabs(psi - 2*M_PI -xhat_.psi) < fabs(psi - xhat_.psi))
  {
    psi -= 2*M_PI;
  }

  // // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE;

  input_cmd.cmd1 = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);  // n_dot
  input_cmd.cmd2 = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi); // e_dot
  input_cmd.cmd3 = pddot_c;                                        // d_dot
  input_cmd.cmd4 = PID_psi_.compute_pid(psi, xhat_.psi, dt_);     // r

  nvel_evel_dvel_yawrate(input_cmd);
}

void ControllerCascadingPID::nvel_evel_dvel_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double x_dot = input_cmd.cmd1;
  double y_dot = input_cmd.cmd2;
  double z_dot = input_cmd.cmd3;
  double r = input_cmd.cmd4;

  // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
  // Rotate body frame velocities to vehicle 1 frame velocities
  double sinp = sin(xhat_.phi);
  double cosp = cos(xhat_.phi);
  double sint = sin(xhat_.theta);
  double cost = cos(xhat_.theta);
  double pxdot =
      cost * xhat_.v_n + sinp * sint * xhat_.v_e + cosp * sint * xhat_.v_d;
  double pydot = cosp * xhat_.v_e - sinp * xhat_.v_d;
  double pddot =
      -sint * xhat_.v_n + sinp * cost * xhat_.v_e + cosp * cost * xhat_.v_d;

  // // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE;

  input_cmd.cmd1 = PID_x_dot_.compute_pid(x_dot, pxdot, dt_);  // ax
  input_cmd.cmd2 = PID_y_dot_.compute_pid(y_dot, pydot, dt_);  // ay
  input_cmd.cmd3 = PID_z_dot_.compute_pid(z_dot, pddot, dt_);  // az
  input_cmd.cmd4 = r;                                         // r

  nacc_eacc_dacc_yawrate(input_cmd);
}

void ControllerCascadingPID::nacc_eacc_dacc_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double equilibrium_throttle = params.get_double("equilibrium_throttle");

  double ax = input_cmd.cmd1;
  double ay = input_cmd.cmd2;
  double az = input_cmd.cmd3;
  double r = input_cmd.cmd4;

  double phi = 0.0;
  double theta = 0.0;

  // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
  double total_acc_c = sqrt((1.0 - az) * (1.0 - az) +
                            ax * ax + ay * ay);  // (in g's)
  if (total_acc_c > 0.001)
  {
    phi = asin(ay / total_acc_c);
    theta = -1.0*asin(ax / total_acc_c);
  }
  else
  {
    phi = 0;
    theta = 0;
  }

  // Compute desired thrust based on current pose
  double cosp = cos(xhat_.phi);
  double cost = cos(xhat_.theta);

  // Save the calculated values to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;

  input_cmd.cmd1 = phi;                                        // phi
  input_cmd.cmd2 = theta;                                      // theta
  input_cmd.cmd3 = r;                                          // r
  input_cmd.cmd4 = (1.0 - az) * equilibrium_throttle / cosp / cost; // throttle

  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::npos_epos_dvel_yaw(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double pn = input_cmd.cmd1;
  double pe = input_cmd.cmd2;
  double vd = input_cmd.cmd3;
  double psi = input_cmd.cmd4;

  // Figure out desired velocities (in inertial frame)
  // By running the position controllers
  double pndot_c = PID_n_.compute_pid(pn, xhat_.position[0], dt_);
  double pedot_c = PID_e_.compute_pid(pe, xhat_.position[1], dt_);

  // Calculate desired yaw rate
  // First, determine the shortest direction to the commanded psi
  if(fabs(psi + 2*M_PI - xhat_.psi) < fabs(psi - xhat_.psi))
  {
    psi += 2*M_PI;
  }
  else if (fabs(psi - 2*M_PI -xhat_.psi) < fabs(psi - xhat_.psi))
  {
    psi -= 2*M_PI;
  }

  // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE;

  input_cmd.cmd1 = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);  // x_dot
  input_cmd.cmd2 = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi); // y_dot
  input_cmd.cmd3 = vd;                                             // z_dot
  input_cmd.cmd4 = PID_psi_.compute_pid(psi, xhat_.psi, dt_);     // r

  nvel_evel_dvel_yawrate(input_cmd);
}

void ControllerCascadingPID::nvel_evel_dpos_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double x_dot = input_cmd.cmd1;
  double y_dot = input_cmd.cmd2;
  double pd = input_cmd.cmd3;
  double r = input_cmd.cmd4;

  // Compute desired accelerations (in terms of g's) in the vehicle 1 frame
  // Rotate body frame velocities to vehicle 1 frame velocities
  double sinp = sin(xhat_.phi);
  double cosp = cos(xhat_.phi);
  double sint = sin(xhat_.theta);
  double cost = cos(xhat_.theta);
  double pxdot =
      cost * xhat_.v_n + sinp * sint * xhat_.v_e + cosp * sint * xhat_.v_d;
  double pydot = cosp * xhat_.v_e - sinp * xhat_.v_d;
  double pddot =
      -sint * xhat_.v_n + sinp * cost * xhat_.v_e + cosp * cost * xhat_.v_d;

  // Nested Loop for Altitude
  double pddot_c = PID_d_.compute_pid(pd, xhat_.position[2], dt_, pddot);

  // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE;

  input_cmd.cmd1 = PID_x_dot_.compute_pid(x_dot, pxdot, dt_);  // ax
  input_cmd.cmd2 = PID_y_dot_.compute_pid(y_dot, pydot, dt_);  // ay
  input_cmd.cmd3 = PID_z_dot_.compute_pid(pddot_c, pddot, dt_);    // az
  input_cmd.cmd4 = r;                                            // r

  nacc_eacc_dacc_yawrate(input_cmd);
}

void ControllerCascadingPID::roll_pitch_yaw_throttle(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // Replace the yaw control command with the calculated yawrate command
  input_cmd.cmd3 = PID_psi_.compute_pid(psi_cmd_, xhat_.psi, dt_, xhat_.r);

  // Assign the correct control mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;

  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::pass_to_firmware_controller(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double min_attitude_altitude = params.get_double("min_attitude_altitude");
  double max_x, max_y, max_z, max_f, min_f;
  max_f = params.get_double("max_throttle");
  min_f = params.get_double("min_throttle");

  // Set the mode and saturation limits
  if (input_cmd.mode == roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH) {
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_PASS_THROUGH;
    max_x = params.get_double("max_roll_torque");
    max_y = params.get_double("max_pitch_torque");
    max_z = params.get_double("max_yaw_torque");
  }
  else if (input_cmd.mode == roscopter_msgs::msg::ControllerCommand::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE) {
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
    max_x = params.get_double("max_phi_dot");
    max_y = params.get_double("max_theta_dot");
    max_z = params.get_double("max_psi_dot");
  }
  else if (input_cmd.mode == roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE) {
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    max_x = params.get_double("max_roll");
    max_y = params.get_double("max_pitch");
    max_z = 2 * M_PI;
  }
  else {
    RCLCPP_WARN_STREAM(this->get_logger(), "Unknown command input type! Using previous command (if any)...");
    max_x = 0.0;
    max_y = 0.0;
    max_z = 0.0;
    return;
  }

  output_cmd_.ignore = rosflight_msgs::msg::Command::IGNORE_NONE;

  // Saturate the commanded output
  output_cmd_.x = saturate(input_cmd.cmd1, max_x, -max_x);
  output_cmd_.y = saturate(input_cmd.cmd2, max_y, -max_y);
  output_cmd_.z = saturate(input_cmd.cmd3, max_z, -max_z);
  output_cmd_.f = saturate(input_cmd.cmd4, max_f, min_f);

  // Check to see if we are above the minimum attitude altitude
  if (-xhat_.position[2] < min_attitude_altitude)
  {
    output_cmd_.x = 0.;
    output_cmd_.y = 0.;
    output_cmd_.z = 0.;
  }
}

}  // namespace controller

