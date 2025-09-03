#include "navigation/path_manager.hpp"

namespace roscopter 
{

// =========== UTILITY FUNCTIONS ===========
double dist(std::array<float, 3> array1, std::array<float, 3> array2)
{
  return sqrt(pow(array1[0] - array2[0], 2) + pow(array1[1] - array2[1], 2) + pow(array1[2] - array2[2], 2));
}

PathManager::PathManager() 
  : PathManagerROS()
  , initial_leg_time_(0.0)
  , T_(0.0)
{
  declare_params();
}

void PathManager::declare_params()
{
  params.declare_double("default_altitude", 10.0);
  params.declare_double("waypoint_tolerance", 1.0);
  params.declare_bool("hold_last", false);
  params.declare_double("max_velocity", 5.0);
  params.declare_double("max_acceleration", 9.81); // meters per second
  params.declare_bool("do_linear_interpolation", false);
}

roscopter_msgs::msg::TrajectoryCommand PathManager::create_default_output()
{
  auto cmd = roscopter_msgs::msg::TrajectoryCommand();
  double default_altitude = params.get_double("default_altitude");
  cmd.position[2] = -default_altitude;
  return cmd;
}

roscopter_msgs::msg::TrajectoryCommand PathManager::manage_path()
{
  // Check the number of waypoints - do we have enough to do path management?
  if (waypoint_list_.size() < 1) {
    output_cmd_ = create_default_output();
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "No waypoints received! Orbiting the origin at " << std::to_string(output_cmd_.position[2]) << " meters!");
    return output_cmd_;
  }

  // Otherwise, manage the correct type of waypoint
  roscopter_msgs::msg::Waypoint curr_wp = waypoint_list_[current_wp_index_];

  if (curr_wp.type == roscopter_msgs::msg::Waypoint::TYPE_GOTO) {
    return manage_goto_wp(curr_wp);
  }
  else {    // Type HOLD
    return manage_hold_wp(curr_wp);
  }
}

roscopter_msgs::msg::TrajectoryCommand PathManager::manage_goto_wp(roscopter_msgs::msg::Waypoint &curr_wp) {
  double waypoint_tolerance = params.get_double("waypoint_tolerance");
  if (dist(xhat_.position, curr_wp.w) <= waypoint_tolerance) {
    // If we are close enough to the target wayoint, increment the index in the waypoint list
    increment_wp_index();
  }

  output_cmd_ = create_trajectory();
  return output_cmd_;
}

roscopter_msgs::msg::TrajectoryCommand PathManager::manage_hold_wp(roscopter_msgs::msg::Waypoint &curr_wp) {
  double waypoint_tolerance = params.get_double("waypoint_tolerance");
  if (dist(xhat_.position, curr_wp.w) <= waypoint_tolerance && !curr_wp.hold_indefinitely) {
    // Start the hold timer, if applicable
    std::chrono::microseconds timer_period_ = std::chrono::microseconds(static_cast<long long>(curr_wp.hold_seconds * 1'000'000));
    if (!timer_started_) {
      hold_timer_ = this->create_wall_timer(timer_period_, std::bind(&PathManager::hold_timer_callback, this));
      timer_started_ = true;
    }
  }

  // Fill in the output command. Note that this is the same regardless of the conditions below.
  output_cmd_ = create_trajectory();
  return output_cmd_;
}

void PathManager::hold_timer_callback() {
  increment_wp_index();

  hold_timer_->cancel();
  timer_started_ = false;
}

void PathManager::increment_wp_index() {
  if (waypoint_list_.size() <= 1) { return; }

  if ((current_wp_index_ + 1) % waypoint_list_.size() == 0) {
    if (!params.get_bool("hold_last")) {
      previous_wp_index_ = current_wp_index_;
      current_wp_index_ = 0;
      initialize_path();
    }
  }
  else {
    previous_wp_index_ = current_wp_index_;
    current_wp_index_++;
    initialize_path();
  }

}

void PathManager::initialize_path()
{
  waypoints_changed_ = true;
  temp_wp_set_ = false;
  initial_leg_time_ = this->get_clock()->now().seconds();

  // Find T_, which is the time it takes to travese waypoints.
  roscopter_msgs::msg::Waypoint curr_wp = waypoint_list_[current_wp_index_];
  double norm = dist(curr_wp.w, prev_wp_.w);
  double max_vel = params.get_double("max_velocity");
  double max_accel = params.get_double("max_acceleration");

  // The constants in the sigma derivatives come from finding the max of those functions
  // on the interval [0,1]
  double T_vel_limited = sigma_prime(0.5) * norm / max_vel;
  double T_acc_limited = sqrt(sigma_double_prime((3-sqrt(3))/6) * norm / max_accel);

  T_ = std::max(T_vel_limited, T_acc_limited);
}

void PathManager::clear_waypoints_internally() {
  waypoint_list_.clear();
  current_wp_index_ = 0;
  path_initialized_ = false;
}

roscopter_msgs::msg::TrajectoryCommand PathManager::create_trajectory()
{
  if (!path_initialized_) {
    initialize_path();
    path_initialized_ = true;
  }

  if (waypoint_list_.size() < 1) {
    // This loop won't work if we have no waypoints
    output_cmd_ = create_default_output();
    return output_cmd_;
  }

  // Compute the control according to Algorithm 15 in Ch. 14 of Beard, McLain textbook
  prev_wp_ = roscopter_msgs::msg::Waypoint();
  roscopter_msgs::msg::Waypoint curr_wp = waypoint_list_[current_wp_index_];
  if (previous_wp_index_ == current_wp_index_) {
    if (!temp_wp_set_) {
      prev_wp_.w = xhat_.position;
      prev_wp_.speed = 0.0;
      prev_wp_.psi = xhat_.psi;
      temp_wp_set_ = true;
    }
  } else {
    prev_wp_ = waypoint_list_[previous_wp_index_];
  }


  if (params.get_bool("do_linear_interpolation")) {
    return linear_interpolation();
  }
  return quintic_interpolation();
}

roscopter_msgs::msg::TrajectoryCommand PathManager::quintic_interpolation()
{
  double tau = get_t() / T_;
  if (tau > 1.0) {
    return output_cmd_;
  }

  double s = sigma(tau);
  double s_p = sigma_prime(tau);
  double s_pp = sigma_double_prime(tau);

  roscopter_msgs::msg::Waypoint curr_wp = waypoint_list_[current_wp_index_];

  // Compute the trajectory commands from a quintic interpolation between points.
  // This guarantees that velocity and acceleration are zero at the endpoints.
  // See the 5th-order smoothstep function on Wikipedia.
  for (int i=0; i<3; ++i) {
    output_cmd_.position[i] = s * curr_wp.w[i] + (1 - s) * prev_wp_.w[i];
    output_cmd_.velocity[i] = s_p * (curr_wp.w[i] - prev_wp_.w[i]) / T_;
    output_cmd_.acceleration[i] = s_pp * (curr_wp.w[i] - prev_wp_.w[i]) / T_ / T_;
  }
  output_cmd_.psi = s * curr_wp.psi + (1 - s) * prev_wp_.psi;
  output_cmd_.psi_dot = s_p * (curr_wp.psi - prev_wp_.psi) / T_;
  output_cmd_.psi_dot_dot = s_pp * (curr_wp.psi - prev_wp_.psi) / T_ / T_;

  return output_cmd_;
}

double PathManager::get_t()
{
  return this->get_clock()->now().seconds() - initial_leg_time_;
}

double PathManager::sigma(double tau)
{
  return 6 * pow(tau, 5.0) - 15 * pow(tau, 4.0) + 10 * pow(tau, 3.0);
}

double PathManager::sigma_prime(double tau)
{
  return 30 * pow(tau, 4.0) - 60 * pow(tau, 3.0) + 30 * pow(tau, 2.0);
}

double PathManager::sigma_double_prime(double tau)
{
  return 120 * pow(tau, 3.0) - 180 * pow(tau, 2.0) + 60 * pow(tau, 1.0);
}

roscopter_msgs::msg::TrajectoryCommand PathManager::linear_interpolation()
{
  roscopter_msgs::msg::Waypoint curr_wp = waypoint_list_[current_wp_index_];
  if (waypoints_changed_) {
    // Reset the path parameters
    double s1 = prev_wp_.speed;
    sigma_(0) = 0;
    sigma_(1) = s1 / dist(curr_wp.w, prev_wp_.w);
    waypoints_changed_ = false;
  }

  if (sigma_(0) >= 1.0) {
    // If we have reached the end of the time window, just return the most recent command.
    return output_cmd_;
  }

  // Integrate the path parameter
  rk4_step();

  // Compute commanded trajectory
  double s_i = curr_wp.speed;
  double s_i1 = prev_wp_.speed;
  double norm = dist(curr_wp.w, prev_wp_.w);
  double sigma_dot_1 = (pow(s_i, 2) - pow(s_i1, 2)) / (2 * pow(norm, 2));

  Eigen::Vector3f difference, position, velocity, acceleration;
  Eigen::Vector3f curr(curr_wp.w.data());
  Eigen::Vector3f prev(prev_wp_.w.data());

  difference = curr - prev;
  position = sigma_(0) * curr + (1 - sigma_(0)) * prev;
  velocity = sigma_(1) * difference;
  acceleration = sigma_dot_1 * difference;

  for (int i=0; i<3; ++i) {
    output_cmd_.position[i] = position[i];
    output_cmd_.velocity[i] = velocity[i];
    output_cmd_.acceleration[i] = acceleration[i];
  }
  output_cmd_.psi = sigma_(0) * curr_wp.psi + (1 - sigma_(0)) * prev_wp_.psi;
  output_cmd_.psi_dot = sigma_(1) * (curr_wp.psi - prev_wp_.psi);
  output_cmd_.psi_dot_dot = 0.0;

  return output_cmd_;

}

void PathManager::rk4_step()
{
  double dt = 1.0 / params.get_double("path_update_frequency");

  Eigen::Vector2f F1 = F(sigma_);
  Eigen::Vector2f F2 = F(sigma_ + dt / 2 * F1);
  Eigen::Vector2f F3 = F(sigma_ + dt / 2 * F2);
  Eigen::Vector2f F4 = F(sigma_ + dt * F3);

  sigma_ = sigma_ + dt / 6 * (F1 + 2*F2 + 2*F3 + F4);
}

Eigen::Vector2f PathManager::F(Eigen::Vector2f sig)
{
  // Rename variables for clarity
  roscopter_msgs::msg::Waypoint current_wp = waypoint_list_[current_wp_index_];

  double s_i = current_wp.speed;
  double s_i1 = prev_wp_.speed;
  double norm = dist(current_wp.w, prev_wp_.w);

  // std::cout << "Norm: " << norm << std::endl;

  Eigen::Vector2f sigma_dot;
  sigma_dot(0) = sig(1);
  sigma_dot(1) = (pow(s_i, 2) - pow(s_i1, 2)) / (2 * pow(norm, 2));

  return sigma_dot;
}

} // namespace roscopter
