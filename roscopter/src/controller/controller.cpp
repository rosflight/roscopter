#include <controller/controller.hpp>

using std::placeholders::_1;

namespace roscopter
{

Controller::Controller() : ControllerROS()
{
  // Calculate max accelerations. Assuming that equilibrium throttle produces
  // 1 g of acceleration and a linear thrust model, these max acceleration
  // values are computed in g's as well.
  double equilibrium_throttle = params.get_double("equilibrium_throttle");
  max_accel_z_ = 1.0 / equilibrium_throttle;
  max_accel_xy_ = sin(acos(equilibrium_throttle)) / equilibrium_throttle / sqrt(2.);

  // Fill out the max_ struct
  this->get_parameter("max_roll", max_.roll);
  this->get_parameter("max_pitch", max_.pitch);
  this->get_parameter("max_yaw_rate", max_.yaw_rate);
  this->get_parameter("max_throttle", max_.throttle);
  this->get_parameter("max_n_dot", max_.n_dot);
  this->get_parameter("max_e_dot", max_.e_dot);
  this->get_parameter("max_d_dot", max_.d_dot);
  this->get_parameter("min_altitude", min_altitude_);

  update_gains();
}

void Controller::update_gains() {
  double P, I, D, tau;
  tau = params.get_double("tau");
  P = params.get_double("x_dot_P");
  I = params.get_double("x_dot_I");
  D = params.get_double("x_dot_D");
  PID_x_dot_.set_gains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = params.get_double("y_dot_P");
  I = params.get_double("y_dot_I");
  D = params.get_double("y_dot_D");
  PID_y_dot_.set_gains(P, I, D, tau, max_accel_xy_, -max_accel_xy_);

  P = params.get_double("z_dot_P");
  I = params.get_double("z_dot_I");
  D = params.get_double("z_dot_D");
  // set max z accelerations so that we can't fall faster than 1 gravity
  PID_z_dot_.set_gains(P, I, D, tau, 1.0, -max_accel_z_);

  P = params.get_double("north_P");
  I = params.get_double("north_I");
  D = params.get_double("north_D");
  max_.n_dot = params.get_double("max_n_dot");
  PID_n_.set_gains(P, I, D, tau, max_.n_dot, -max_.n_dot);

  P = params.get_double("east_P");
  I = params.get_double("east_I");
  D = params.get_double("east_D");
  max_.e_dot = params.get_double("max_e_dot");
  PID_e_.set_gains(P, I, D, tau, max_.e_dot, -max_.e_dot);

  P = params.get_double("down_P");
  I = params.get_double("down_I");
  D = params.get_double("down_D");
  max_.d_dot = params.get_double("max_d_dot");
  PID_d_.set_gains(P, I, D, tau, max_.d_dot, -max_.d_dot);

  P = params.get_double("psi_P");
  I = params.get_double("psi_I");
  D = params.get_double("psi_D");
  PID_psi_.set_gains(P, I, D, tau);

  max_.roll = params.get_double("max_roll");
  max_.pitch = params.get_double("max_pitch");
  max_.yaw_rate = params.get_double("max_yaw_rate");
  max_.throttle = params.get_double("max_throttle");

  max_.n_dot = params.get_double("max_n_dot");
  max_.e_dot = params.get_double("max_e_dot");
  max_.d_dot = params.get_double("max_d_dot");
}


rosflight_msgs::msg::Command Controller::compute_control(roscopter_msgs::msg::ControllerCommand & input_cmd, double dt)
{
  // Define parameters here for clarity
  double equilibrium_throttle = params.get_double("equilibrium_throttle");

  uint8_t mode_flag = input_cmd.mode;
  roscopter_msgs::msg::ControllerCommand transition_cmd = input_cmd;

  if(dt <= 0.0000001) {
    // This messes up the derivative calculation in the PID controllers
    RCLCPP_WARN_STREAM(this->get_logger(), "dt < 0.0000001");
    // TODO: Do we need to keep the last cmd and return it here?
    return output_cmd_;
  }

  if(mode_flag == roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DPOS_YAW)
  {
    double pn = transition_cmd.cmd1;
    double pe = transition_cmd.cmd2;
    double pd = transition_cmd.cmd3;
    double psi = transition_cmd.cmd4;

    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_n_.compute_pid(pn, xhat_.position[0], dt);
    double pedot_c = PID_e_.compute_pid(pe, xhat_.position[1], dt);
    double pddot_c = PID_d_.compute_pid(pd, xhat_.position[2], dt);

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
    mode_flag = roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE;

    transition_cmd.cmd1 = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);  // x_dot
    transition_cmd.cmd2 = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi); // y_dot
    transition_cmd.cmd3 = pddot_c;                                        // z_dot
    transition_cmd.cmd4 = PID_psi_.compute_pid(psi, xhat_.psi, dt);     // r
  }

  if(mode_flag == roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DVEL_YAW)
  {
    double pn = transition_cmd.cmd1;
    double pe = transition_cmd.cmd2;
    double vd = transition_cmd.cmd3;
    double psi = transition_cmd.cmd4;

    // Figure out desired velocities (in inertial frame)
    // By running the position controllers
    double pndot_c = PID_n_.compute_pid(pn, xhat_.position[0], dt);
    double pedot_c = PID_e_.compute_pid(pe, xhat_.position[1], dt);

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
    mode_flag = roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE;

    transition_cmd.cmd1 = pndot_c*cos(xhat_.psi) + pedot_c*sin(xhat_.psi);  // x_dot
    transition_cmd.cmd2 = -pndot_c*sin(xhat_.psi) + pedot_c*cos(xhat_.psi); // y_dot
    transition_cmd.cmd3 = vd;                                             // z_dot
    transition_cmd.cmd4 = PID_psi_.compute_pid(psi, xhat_.psi, dt);     // r
  }

  if(mode_flag == roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE)
  {
    double x_dot = transition_cmd.cmd1;
    double y_dot = transition_cmd.cmd2;
    double z_dot = transition_cmd.cmd3;
    double r = transition_cmd.cmd4;

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

    // Save the calculated velocities to the command and change to the appropriate mode
    mode_flag = roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE;

    transition_cmd.cmd1 = PID_x_dot_.compute_pid(x_dot, pxdot, dt);  // ax
    transition_cmd.cmd2 = PID_y_dot_.compute_pid(y_dot, pydot, dt);  // ay
    transition_cmd.cmd3 = PID_z_dot_.compute_pid(z_dot, pddot, dt);  // az
    transition_cmd.cmd4 = r;                                        // r
  }

  if(mode_flag == roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DPOS_YAWRATE)
  {
    double x_dot = transition_cmd.cmd1;
    double y_dot = transition_cmd.cmd2;
    double pd = transition_cmd.cmd3;
    double r = transition_cmd.cmd4;

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
    double pddot_c = PID_d_.compute_pid(pd, xhat_.position[2], dt, pddot);

    // Save the calculated velocities to the command and change to the appropriate mode
    mode_flag = roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE;

    transition_cmd.cmd1 = PID_x_dot_.compute_pid(x_dot, pxdot, dt);  // ax
    transition_cmd.cmd2 = PID_y_dot_.compute_pid(y_dot, pydot, dt);  // ay
    transition_cmd.cmd3 = PID_z_dot_.compute_pid(pddot_c, pddot, dt);    // az
    transition_cmd.cmd4 = r;                                            // r
  }

  if(mode_flag == roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE)
  {
    double ax = transition_cmd.cmd1;
    double ay = transition_cmd.cmd2;
    double az = transition_cmd.cmd3;
    double r = transition_cmd.cmd4;

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

    // Save the calculated velocities to the command and change to the appropriate mode
    mode_flag = roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE;

    transition_cmd.cmd1 = phi;                                        // phi
    transition_cmd.cmd2 = theta;                                      // theta
    transition_cmd.cmd3 = r;                                          // r
    transition_cmd.cmd4 = (1.0 - az) * equilibrium_throttle / cosp / cost; // throttle
  }

  if(mode_flag == roscopter_msgs::msg::ControllerCommand::MODE_ROLL_PITCH_YAWRATE_THROTTLE)
  {
    double phi = transition_cmd.cmd1;
    double theta = transition_cmd.cmd2;
    double r = transition_cmd.cmd3;
    double throttle = transition_cmd.cmd4;

    // Saturate and send the command
    output_cmd_.mode = rosflight_msgs::msg::Command::MODE_ROLL_PITCH_YAWRATE_THROTTLE;
    output_cmd_.ignore = rosflight_msgs::msg::Command::IGNORE_NONE;

    output_cmd_.x = saturate(phi, max_.roll, -max_.roll);       // roll
    output_cmd_.y = saturate(theta, max_.pitch, -max_.pitch);   // pitch
    output_cmd_.z = saturate(r, max_.yaw_rate, -max_.yaw_rate); // yawrate
    output_cmd_.f = saturate(throttle, max_.throttle, 0.0);     // throttle

    if (-xhat_.position[2] < min_altitude_)
    {
      output_cmd_.x = 0.;   // roll
      output_cmd_.y = 0.;   // pitch
      output_cmd_.z = 0.;   // yawrate
    }
  }

  // TODO: Do we need to check if the output is all zeros? i.e., if 
  // loop doesn't make it to the last section where it gets packaged up...
  return output_cmd_;
}

void Controller::reset_integrators()
{
  PID_x_dot_.clear_integrator();
  PID_y_dot_.clear_integrator();
  PID_z_dot_.clear_integrator();
  PID_n_.clear_integrator();
  PID_e_.clear_integrator();
  PID_d_.clear_integrator();
  PID_psi_.clear_integrator();
}

}  // namespace controller
