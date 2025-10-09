#include <controller/controller_cascading_pid.hpp>

using std::placeholders::_1;

namespace roscopter
{

ControllerCascadingPID::ControllerCascadingPID() : ControllerStateMachine(), params_initialized_(false)
{
  // Declare and set parameters associated with Cascading PID controller
  declare_params();
  params.set_parameters();

  // Check to make sure equilibrium throttle is set properly
  if (!params.get_double("equilibrium_throttle")) {
    RCLCPP_ERROR(this->get_logger(), "Controller MAV equilibrium throttle not found!");
  }

  update_gains();
  reset_integrators();

  params_initialized_ = true;
  // Update gains after all the parameters have been set to initialize the PID controllers
  update_gains();
}

void ControllerCascadingPID::declare_params() {
  params.declare_double("roll_to_torque_kp", 0.2);
  params.declare_double("roll_to_torque_ki", 0.0);
  params.declare_double("roll_to_torque_kd", 0.2);

  params.declare_double("pitch_to_torque_kp", 0.2);
  params.declare_double("pitch_to_torque_ki", 0.0);
  params.declare_double("pitch_to_torque_kd", 0.2);

  params.declare_double("yaw_to_torque_kp", 0.2);
  params.declare_double("yaw_to_torque_ki", 0.002);
  params.declare_double("yaw_to_torque_kd", 0.2);

  params.declare_double("roll_rate_to_torque_kp", 0.2);
  params.declare_double("roll_rate_to_torque_ki", 0.0);
  params.declare_double("roll_rate_to_torque_kd", 0.2);

  params.declare_double("pitch_rate_to_torque_kp", 0.2);
  params.declare_double("pitch_rate_to_torque_ki", 0.0);
  params.declare_double("pitch_rate_to_torque_kd", 0.2);

  params.declare_double("yaw_rate_to_torque_kp", 0.2);
  params.declare_double("yaw_rate_to_torque_ki", 0.0);
  params.declare_double("yaw_rate_to_torque_kd", 0.2);

  params.declare_double("yaw_to_rate_kp", 0.2);
  params.declare_double("yaw_to_rate_ki", 0.0);
  params.declare_double("yaw_to_rate_kd", 0.2);

  params.declare_double("vel_n_to_accel_kp", 0.5);
  params.declare_double("vel_n_to_accel_ki", 0.0);
  params.declare_double("vel_n_to_accel_kd", 0.05);

  params.declare_double("vel_e_to_accel_kp", 0.5);
  params.declare_double("vel_e_to_accel_ki", 0.0);
  params.declare_double("vel_e_to_accel_kd", 0.05);

  params.declare_double("vel_d_to_accel_kp", 0.4);
  params.declare_double("vel_d_to_accel_ki", 0.25);
  params.declare_double("vel_d_to_accel_kd", 0.1);

  params.declare_double("pos_n_to_vel_kp", 1.0);
  params.declare_double("pos_n_to_vel_ki", 0.1);
  params.declare_double("pos_n_to_vel_kd", 0.35);

  params.declare_double("pos_e_to_vel_kp", 1.0);
  params.declare_double("pos_e_to_vel_ki", 0.1);
  params.declare_double("pos_e_to_vel_kd", 0.2);

  params.declare_double("pos_d_to_vel_kp", 1.0);
  params.declare_double("pos_d_to_vel_ki", 0.0);
  params.declare_double("pos_d_to_vel_kd", 0.0);

  // System parameters
  params.declare_double("tau", 0.05);
  params.declare_double("mass", 2.0);
  params.declare_double("gravity", 9.81);

  // Saturation Limit parameters
  params.declare_double("max_roll_deg", 45.0);  
  params.declare_double("max_pitch_deg", 45.0);

  params.declare_double("max_roll_rate_deg", 1.0);
  params.declare_double("max_pitch_rate_deg", 1.0);
  params.declare_double("max_yaw_rate_deg", 1.0);

  params.declare_double("max_roll_torque", 10.0);
  params.declare_double("max_pitch_torque", 10.0);
  params.declare_double("max_yaw_torque", 10.0);

  params.declare_double("max_descend_accel", 1.0);
  params.declare_double("max_descend_rate", 3.0);

  params.declare_double("equilibrium_throttle", 0.45); 
  params.declare_double("max_throttle", 0.85);
  params.declare_double("min_throttle", 0.05);

  params.declare_double("min_altitude_for_attitude_ctrl", 0.15);
}

void ControllerCascadingPID::update_gains() {
  // Don't update gains if parameters are not initialized.
  if (!params_initialized_) { return; }

  RCLCPP_INFO_STREAM(this->get_logger(), "Updating gains!");

  double P, I, D, tau, max;
  tau = params.get_double("tau");

  // Roll to torque PID loop
  P = params.get_double("roll_to_torque_kp");
  I = params.get_double("roll_to_torque_ki");
  D = params.get_double("roll_to_torque_kd");
  // TODO: Do these max torque parameters make sense?
  // Should we just let the max angle parameters take care of it?
  max = params.get_double("max_roll_torque");
  PID_roll_to_torque_.set_gains(P, I, D, tau, max, -max);

  // Pitch to torque PID loop
  P = params.get_double("pitch_to_torque_kp");
  I = params.get_double("pitch_to_torque_ki");
  D = params.get_double("pitch_to_torque_kd");
  max = params.get_double("max_pitch_torque");
  PID_pitch_to_torque_.set_gains(P, I, D, tau, max, -max);

  // Yaw to torque PID loop
  P = params.get_double("yaw_to_torque_kp");
  I = params.get_double("yaw_to_torque_ki");
  D = params.get_double("yaw_to_torque_kd");
  max = params.get_double("max_yaw_torque");
  PID_yaw_to_torque_.set_gains(P, I, D, tau, max, -max);

  // Roll rate to torque PID loop
  P = params.get_double("roll_rate_to_torque_kp");
  I = params.get_double("roll_rate_to_torque_ki");
  D = params.get_double("roll_rate_to_torque_kd");
  max = params.get_double("max_roll_torque");
  PID_roll_rate_to_torque_.set_gains(P, I, D, tau, max, -max);

  // Pitch rate to torque PID loop
  P = params.get_double("pitch_rate_to_torque_kp");
  I = params.get_double("pitch_rate_to_torque_ki");
  D = params.get_double("pitch_rate_to_torque_kd");
  max = params.get_double("max_pitch_torque");
  PID_pitch_rate_to_torque_.set_gains(P, I, D, tau, max, -max);

  // Yaw rate to torque PID loop
  P = params.get_double("yaw_rate_to_torque_kp");
  I = params.get_double("yaw_rate_to_torque_ki");
  D = params.get_double("yaw_rate_to_torque_kd");
  max = params.get_double("max_yaw_torque");
  PID_yaw_rate_to_torque_.set_gains(P, I, D, tau, max, -max);

  // PID loop from yaw to yaw rate
  P = params.get_double("yaw_to_rate_kp");
  I = params.get_double("yaw_to_rate_ki");
  D = params.get_double("yaw_to_rate_kd");
  max = params.get_double("max_yaw_rate_deg");
  PID_yaw_to_rate_.set_gains(P, I, D, tau, max, -max);

  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
  double equilibrium_throttle = params.get_double("equilibrium_throttle");
  double max_accel_z = 1.0 / equilibrium_throttle;

  // North velocity to accel PID loop
  P = params.get_double("vel_n_to_accel_kp");
  I = params.get_double("vel_n_to_accel_ki");
  D = params.get_double("vel_n_to_accel_kd");
  PID_vel_n_to_accel_.set_gains(P, I, D, tau);

  // East velocity to accel PID loop
  P = params.get_double("vel_e_to_accel_kp");
  I = params.get_double("vel_e_to_accel_ki");
  D = params.get_double("vel_e_to_accel_kd");
  PID_vel_e_to_accel_.set_gains(P, I, D, tau);

  // Down velocity to accel PID loop
  P = params.get_double("vel_d_to_accel_kp");
  I = params.get_double("vel_d_to_accel_ki");
  D = params.get_double("vel_d_to_accel_kd");
  // set max z accelerations so that we can't fall faster than 1 gravity
  max = max_accel_z;
  double min = params.get_double("max_descend_accel");
  PID_vel_d_to_accel_.set_gains(P, I, D, tau, min, -max);

  // North position to velocity PID loop
  P = params.get_double("pos_n_to_vel_kp");
  I = params.get_double("pos_n_to_vel_ki");
  D = params.get_double("pos_n_to_vel_kd");
  PID_n_to_vel_.set_gains(P, I, D, tau);

  // East position to velocity PID loop
  P = params.get_double("pos_e_to_vel_kp");
  I = params.get_double("pos_e_to_vel_ki");
  D = params.get_double("pos_e_to_vel_kd");
  PID_e_to_vel_.set_gains(P, I, D, tau);

  // Down position to velocity PID loop
  P = params.get_double("pos_d_to_vel_kp");
  I = params.get_double("pos_d_to_vel_ki");
  D = params.get_double("pos_d_to_vel_kd");
  // TODO: This parameter is confusing since it doesn't always apply. It only applies when you are inserting
  // above the velocity controller. Commanding accels or angles means this parameter is bypassed.
  // What do we do to saturate the down command? Do we leave it up to the min throttle parameter?
  // min = params.get_double("max_descend_rate");
  // PID_d_to_vel_.set_gains(P, I, D, tau, min, -DBL_MAX);
  PID_d_to_vel_.set_gains(P, I, D, tau);
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

    case roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAW_THROTTLE:
      roll_pitch_yaw_throttle(input_cmd);
      break;

    // Sends commands directly to firmware mixer
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAW_THRUST_TO_MIXER:
      roll_pitch_yaw_thrust_to_motor(input_cmd);
      break;

    // Sends commands directly to firmware mixer
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THRUST_TO_MIXER:
      roll_pitch_yawrate_thrust_to_motor(input_cmd);
      break;

    // Sends commands directly to firmware mixer
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLLRATE_PITCHRATE_YAWRATE_THRUST_TO_MIXER:
      rollrate_pitchrate_yawrate_thrust_to_motor(input_cmd);
      break;

    // Sends commands directly to firmware mixer
    case roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH_TO_MIXER:
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE:
    case roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE:
      pass_to_firmware_controller(input_cmd);
      break;

    default:
      // TODO: Is this desired behavior?
      RCLCPP_ERROR_STREAM(this->get_logger(), "Unknown control mode! Using previous command (if any)...");
      break;
  }

  return output_cmd_;
}

void ControllerCascadingPID::roll_pitch_yaw_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // Saturate the input commands (since we go directly to thrusts).
  double max_phi = params.get_double("max_roll_deg") * TO_RADIANS;
  double max_theta = params.get_double("max_pitch_deg") * TO_RADIANS;
  double phi = saturate(input_cmd.cmd1, max_phi, -max_phi);
  double theta = saturate(input_cmd.cmd2, max_theta, -max_theta);
  double psi = input_cmd.cmd3;

  // Update input command values (except thrust)
  input_cmd.cmd1 = PID_roll_to_torque_.compute_pid(phi, xhat_.phi, dt_, xhat_.p);
  input_cmd.cmd2 = PID_pitch_to_torque_.compute_pid(theta, xhat_.theta, dt_, xhat_.q);
  input_cmd.cmd3 = PID_yaw_to_torque_.compute_pid(psi, xhat_.psi, dt_, xhat_.r);

  // Update the mode with the correct type
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH_TO_MIXER;
  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::roll_pitch_yawrate_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // Saturate the input commands (since we go directly to thrusts).
  double max_phi = params.get_double("max_roll_deg") * TO_RADIANS;
  double max_theta = params.get_double("max_pitch_deg") * TO_RADIANS;
  double max_yaw_rate = params.get_double("max_yaw_rate_deg") * TO_RADIANS;
  double phi = saturate(input_cmd.cmd1, max_phi, -max_phi);
  double theta = saturate(input_cmd.cmd2, max_theta, -max_theta);
  double r = saturate(input_cmd.cmd3, max_yaw_rate, -max_yaw_rate);

  // Update input command values (except thrust)
  input_cmd.cmd1 = PID_roll_to_torque_.compute_pid(phi, xhat_.phi, dt_, xhat_.p);
  input_cmd.cmd2 = PID_pitch_to_torque_.compute_pid(theta, xhat_.theta, dt_, xhat_.q);
  input_cmd.cmd3 = PID_yaw_rate_to_torque_.compute_pid(r, xhat_.r, dt_);

  // Update the mode with the correct type
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH_TO_MIXER;
  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::rollrate_pitchrate_yawrate_thrust_to_motor(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double max_roll_rate = params.get_double("max_roll_rate_deg") * TO_RADIANS;
  double max_pitch_rate = params.get_double("max_pitch_rate_deg") * TO_RADIANS;
  double max_yaw_rate = params.get_double("max_yaw_rate_deg") * TO_RADIANS;
  double p = saturate(input_cmd.cmd1, max_roll_rate, -max_roll_rate);
  double q = saturate(input_cmd.cmd2, max_pitch_rate, -max_pitch_rate);
  double r = saturate(input_cmd.cmd3, max_yaw_rate, -max_yaw_rate);

  // Update input command values
  input_cmd.cmd1 = PID_roll_rate_to_torque_.compute_pid(p , xhat_.p, dt_);
  input_cmd.cmd2 = PID_pitch_rate_to_torque_.compute_pid(q, xhat_.q, dt_);
  input_cmd.cmd3 = PID_yaw_rate_to_torque_.compute_pid(r, xhat_.r, dt_);

  // Update the mode with the correct type
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH_TO_MIXER;
  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::npos_epos_dpos_yaw(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double pn = input_cmd.cmd1;
  double pe = input_cmd.cmd2;
  double pd = input_cmd.cmd3;
  double psi = input_cmd.cmd4;

  // First, determine the shortest direction to the commanded psi (wrap to within 180)
  psi = wrap_within_180(xhat_.psi, psi);

  // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE;

  // Inertial velocities in inertial NED frame
  input_cmd.cmd1 = PID_n_to_vel_.compute_pid(pn, xhat_.position[0], dt_);
  input_cmd.cmd2 = PID_e_to_vel_.compute_pid(pe, xhat_.position[1], dt_);
  input_cmd.cmd3 = PID_d_to_vel_.compute_pid(pd, xhat_.position[2], dt_);
  input_cmd.cmd4 = PID_yaw_to_rate_.compute_pid(psi, xhat_.psi, dt_);         // r in vehicle-1 frame

  nvel_evel_dvel_yawrate(input_cmd);
}

void ControllerCascadingPID::nvel_evel_dvel_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // Expressed in inertial frame
  double vel_n = input_cmd.cmd1;
  double vel_e = input_cmd.cmd2;
  double vel_d = input_cmd.cmd3;
  double r = input_cmd.cmd4;

  // Rotate estimated velocities into inertial frame
  Eigen::Quaterniond q_body_to_inertial(xhat_.quat[0], xhat_.quat[1], xhat_.quat[2], xhat_.quat[3]);
  Eigen::Vector3d v_body(xhat_.v_n, xhat_.v_e, xhat_.v_d);
  Eigen::Vector3d v_inertial = q_body_to_inertial * v_body;

  // Compute desired accelerations (in terms of g's) in the inertial frame
  double a_n = PID_vel_n_to_accel_.compute_pid(vel_n, v_inertial(0), dt_);  // ax
  double a_e = PID_vel_e_to_accel_.compute_pid(vel_e, v_inertial(1), dt_);  // ay
  double a_d = PID_vel_d_to_accel_.compute_pid(vel_d, v_inertial(2), dt_);  // az

  // Rotate inertial frame accelerations to vehicle 1 frame accelerations
  double sin_psi = sin(xhat_.psi);
  double cos_psi = cos(xhat_.psi);
  
  double a_n_vehicle1 = cos_psi * a_n + sin_psi * a_e;
  double a_e_vehicle1 = -sin_psi * a_n + cos_psi * a_e;
  double a_d_vehicle1 = a_d;

  // Save the calculated accels to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE;
  input_cmd.cmd1 = a_n_vehicle1;
  input_cmd.cmd2 = a_e_vehicle1;
  input_cmd.cmd3 = a_d_vehicle1;
  input_cmd.cmd4 = r;

  nacc_eacc_dacc_yawrate(input_cmd);
}

void ControllerCascadingPID::nacc_eacc_dacc_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double equilibrium_throttle = params.get_double("equilibrium_throttle");
  double max_roll_deg = params.get_double("max_roll_deg");
  double max_pitch_deg = params.get_double("max_pitch_deg");

  // In vehicle-1 frame, units of g's
  double ax = input_cmd.cmd1;
  double ay = input_cmd.cmd2;
  double az = input_cmd.cmd3;
  double r = input_cmd.cmd4;

  double phi = 0.0;
  double theta = 0.0;

  // Model inversion (m[ax;ay;az] = m[0;0;g] + R'[0;0;-T]
  double total_acc_c = sqrt((1.0 - az) * (1.0 - az) +
                            ax * ax + ay * ay);  // (in g's)
  // Avoid dividing by zero
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

  // Compute desired thrust based on current pose (with mass divided out)
  double cosp = cos(xhat_.phi);
  double cost = cos(xhat_.theta);
  double desired_accel = (1.0 - az) / cosp / cost;

  // Save the calculated values to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;

  // TODO: We are saturating twice, once here, once later. Fix this
  input_cmd.cmd1 = saturate(phi, max_roll_deg, -max_roll_deg);         // phi
  input_cmd.cmd2 = saturate(theta, max_pitch_deg, -max_pitch_deg);     // theta
  input_cmd.cmd3 = r;                                          // r
  input_cmd.cmd4 = desired_accel * equilibrium_throttle;      // throttle

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
  double pndot_c = PID_n_to_vel_.compute_pid(pn, xhat_.position[0], dt_);
  double pedot_c = PID_e_to_vel_.compute_pid(pe, xhat_.position[1], dt_);

  // First, determine the shortest direction to the commanded psi (wrap within 180)
  psi = wrap_within_180(xhat_.psi, psi);

  // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE;

  // Expressed in inertial frame
  input_cmd.cmd1 = pndot_c;
  input_cmd.cmd2 = pedot_c;
  input_cmd.cmd3 = vd;
  input_cmd.cmd4 = PID_yaw_to_rate_.compute_pid(psi, xhat_.psi, dt_);

  nvel_evel_dvel_yawrate(input_cmd);
}

void ControllerCascadingPID::nvel_evel_dpos_yawrate(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  double vel_n = input_cmd.cmd1;
  double vel_e = input_cmd.cmd2;
  double pd = input_cmd.cmd3;
  double r = input_cmd.cmd4;

  // Save the calculated velocities to the command and change to the appropriate mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE;

  input_cmd.cmd1 = vel_n;
  input_cmd.cmd2 = vel_e;
  input_cmd.cmd3 = PID_d_to_vel_.compute_pid(pd, xhat_.position[2], dt_);
  input_cmd.cmd4 = r;

  nvel_evel_dvel_yawrate(input_cmd);
}

void ControllerCascadingPID::roll_pitch_yaw_throttle(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // Replace the yaw control command with the calculated yawrate command
  double yaw_cmd = input_cmd.cmd3;
  input_cmd.cmd3 = PID_yaw_to_rate_.compute_pid(yaw_cmd, xhat_.psi, dt_, xhat_.r);

  // Assign the correct control mode
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;

  pass_to_firmware_controller(input_cmd);
}

void ControllerCascadingPID::pass_to_firmware_controller(roscopter_msgs::msg::ControllerCommand input_cmd)
{
  // Saturate and return the output command object
  double min_altitude_for_attitude_ctrl = params.get_double("min_altitude_for_attitude_ctrl");
  double max_x, max_y, max_z, max_f, min_f;
  max_f = params.get_double("max_throttle");
  min_f = params.get_double("min_throttle");

  // Set the mode and saturation limits
  if (input_cmd.mode == roscopter_msgs::msg::ControllerCommand::MODE_PASS_THROUGH_TO_MIXER) {
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_PASS_THROUGH;
    max_x = params.get_double("max_roll_torque");
    max_y = params.get_double("max_pitch_torque");
    max_z = params.get_double("max_yaw_torque");

    // Compute the max and min force based on the equilibrium throttle
    double equilibrium_throttle = params.get_double("equilibrium_throttle");
    double mass = params.get_double("mass");
    double gravity = params.get_double("gravity");
    min_f = -mass * gravity / equilibrium_throttle * max_f; // Negative since NED
    max_f = -mass * gravity / equilibrium_throttle * min_f;
    input_cmd.cmd4 *= -1; // Thrust in the NED frame is negative
  }
  else if (input_cmd.mode == roscopter_msgs::msg::ControllerCommand::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE) {
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_ROLLRATE_PITCHRATE_YAWRATE_THROTTLE;
    max_x = params.get_double("max_roll_rate_deg");
    max_y = params.get_double("max_pitch_rate_deg");
    max_z = params.get_double("max_yaw_rate_deg");
  }
  else if (input_cmd.mode == roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE) {
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    max_x = params.get_double("max_roll_deg");
    max_y = params.get_double("max_pitch_deg");
    max_z = params.get_double("max_yaw_rate_deg");
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
  output_cmd_.qx = saturate(input_cmd.cmd1, max_x, -max_x);
  output_cmd_.qy = saturate(input_cmd.cmd2, max_y, -max_y);
  output_cmd_.qz = saturate(input_cmd.cmd3, max_z, -max_z);
  output_cmd_.fz = saturate(input_cmd.cmd4, max_f, min_f);
  output_cmd_.fx = 0.0;
  output_cmd_.fy = 0.0;

  // Check to see if we are above the minimum attitude altitude
  if (abs(xhat_.position[2]) < min_altitude_for_attitude_ctrl)
  {
    output_cmd_.qx = 0.;
    output_cmd_.qy = 0.;
    output_cmd_.qz = 0.;
  }
}

void ControllerCascadingPID::reset_integrators()
{
  PID_vel_n_to_accel_.clear_integrator();
  PID_vel_e_to_accel_.clear_integrator();
  PID_vel_d_to_accel_.clear_integrator();

  PID_n_to_vel_.clear_integrator();
  PID_e_to_vel_.clear_integrator();
  PID_d_to_vel_.clear_integrator();

  PID_roll_to_torque_.clear_integrator();
  PID_pitch_to_torque_.clear_integrator();
  PID_yaw_to_torque_.clear_integrator();

  PID_roll_rate_to_torque_.clear_integrator();
  PID_pitch_rate_to_torque_.clear_integrator();
  PID_yaw_rate_to_torque_.clear_integrator();
}

}  // namespace controller

