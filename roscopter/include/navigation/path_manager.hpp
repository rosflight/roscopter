#ifndef PATH_MANAGER_H
#define PATH_MANAGER_H

#include <algorithm>
#include <vector>
#include <cmath>

#include "navigation/path_manager_ros.hpp"
#include "roscopter_msgs/msg/waypoint.hpp"
#include "roscopter_msgs/msg/trajectory_command.hpp"

#include <Eigen/Core>

namespace roscopter {

class PathManager : public PathManagerROS
{
public:
  PathManager();

private:
  // Methods
  roscopter_msgs::msg::TrajectoryCommand manage_path() override;
  void declare_params();
  roscopter_msgs::msg::TrajectoryCommand manage_goto_wp(roscopter_msgs::msg::Waypoint &curr_wp);
  roscopter_msgs::msg::TrajectoryCommand manage_hold_wp(roscopter_msgs::msg::Waypoint &curr_wp);
  roscopter_msgs::msg::TrajectoryCommand create_trajectory();
  roscopter_msgs::msg::TrajectoryCommand create_default_output();
  roscopter_msgs::msg::TrajectoryCommand quintic_interpolation();
  roscopter_msgs::msg::TrajectoryCommand linear_interpolation();
  void hold_timer_callback();
  void increment_wp_index();
  void clear_waypoints_internally() override;
  void rk4_step();
  Eigen::Vector2f F(Eigen::Vector2f sig);
  void initialize_path();
  double get_t();
  double sigma(double tau);
  double sigma_prime(double tau);
  double sigma_double_prime(double tau);

  // Member variables
  rclcpp::TimerBase::SharedPtr hold_timer_;
  roscopter_msgs::msg::TrajectoryCommand output_cmd_;
  roscopter_msgs::msg::Waypoint prev_wp_;
  double T_; // Time for each leg of the waypoint path
  bool temp_wp_set_ = false;
  bool path_initialized_ = false;
  bool timer_started_ = false;
  int current_wp_index_ = 0;
  int previous_wp_index_ = 0;
  bool waypoints_changed_ = true;
  Eigen::Vector2f sigma_;
  double initial_leg_time_;
};

} // namespace roscopter

#endif
