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
{
  declare_params();
}

void PathManager::declare_params()
{
  params.declare_double("default_altitude", 10.0);
  params.declare_double("waypoint_tolerance", 1.0);
  params.declare_bool("hold_last", false);
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
  } else if (waypoint_list_.size() == 1) {
    // If only one waypoint is added, do a HOLD type trajectory from the current position
    roscopter_msgs::msg::Waypoint wp = waypoint_list_[0];
    return manage_goto_wp(wp);
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

  // std::cout << "waypoint list size: " << waypoint_list_.size() << std::endl;

  if ((current_wp_index_ + 1) % waypoint_list_.size() == 0) {
    if (!params.get_bool("hold_last")) {
      previous_wp_index_ = current_wp_index_;
      current_wp_index_ = 0;
      waypoints_changed_ = true;
      temp_wp_set_ = false;
    }
  }
  else {
    previous_wp_index_ = current_wp_index_;
    current_wp_index_++;
    waypoints_changed_ = true;
    temp_wp_set_ = false;
  }
}

void PathManager::clear_waypoints_internally() {
  waypoint_list_.clear();
  current_wp_index_ = 0;
}

roscopter_msgs::msg::TrajectoryCommand PathManager::create_trajectory()
{
  // Assume that t = t_0 (i.e. fly straight at the waypoint, no matter where you are.)
  // roscopter_msgs::msg::TrajectoryCommand output_cmd;
  //
  // Eigen::Vector3f curr_pos, wp_pos, vel_vect;
  // curr_pos << xhat_.position[0], xhat_.position[1], xhat_.position[2];
  // wp_pos << curr_wp.w[0], curr_wp.w[1], curr_wp.w[2];
  // vel_vect = curr_wp.speed * (wp_pos - curr_pos) / (wp_pos - curr_pos).norm();
  //
  // output_cmd.position = curr_wp.w;
  // output_cmd.velocity = {vel_vect[0], vel_vect[1], vel_vect[2]};
  // output_cmd.acceleration = {0,0,0};
  // output_cmd.psi = curr_wp.psi;
  // output_cmd.psi_dot = 0.0;
  // output_cmd.psi_dot_dot = 0.0;
  // return output_cmd;

  if (waypoint_list_.size() < 1) {
    // This loop won't work if we have no waypoints
    output_cmd_ = create_default_output();
    return output_cmd_;
  }

  // std::cout << "prev and curr wp: " << previous_wp_index_ << " " << current_wp_index_ << std::endl;

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
    // std::cout << "Prev: "; for (int i=0; i<3; ++i) { std::cout << prev_wp_.w[i] << " "; } std::cout << std::endl;
    // std::cout << "Curr: "; for (int i=0; i<3; ++i) { std::cout << curr_wp.w[i] << " "; } std::cout << std::endl;
  }

  if (waypoints_changed_) {
    // Reset the path parameters
    double s1 = prev_wp_.speed;
    sigma_(0) = 0;
    sigma_(1) = s1 / dist(curr_wp.w, prev_wp_.w);
    // std::cout << "prevptr and curr_ptr: " << previous_wp_index_ << " " << current_wp_index_ << std::endl;
    // std::cout << "Prev: "; for (int i=0; i<3; ++i) { std::cout << prev_wp_.w[i] << " "; } std::cout << std::endl;
    // std::cout << "Curr: "; for (int i=0; i<3; ++i) { std::cout << curr_wp.w[i] << " "; } std::cout << std::endl;
    // std::cout << "Sigma: " << sigma_(0) << std::endl;
    waypoints_changed_ = false;
  }

  if (sigma_(0) >= 1.0) {
    // If we have reached the end of the time window, just return the most recent command.
    return output_cmd_;
    // sigma_(0) = 1.0;
    // sigma_(1) = curr_wp.speed / dist(curr_wp.w, prev_wp_.w);
  }

  // Integrate the path parameter
  // std::cout << "Sigmas(pre rk4): " << sigma_(0) << " " << sigma_(1) << std::endl;
  rk4_step();
  // std::cout << "Sigmas(post rk4): " << sigma_(0) << " " << sigma_(1) << std::endl;

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

  // std::cout << "Sigmas: " << sigma_(0) << " " << sigma_(1) << std::endl;
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
