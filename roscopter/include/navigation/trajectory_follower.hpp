#ifndef TRAJECTORY_FOLLOWER_HPP
#define TRAJECTORY_FOLLOWER_HPP

#include "navigation/trajectory_follower_ros.hpp"
#include "controller/simple_pid.hpp"
#include <stdint.h>

namespace roscopter
{

class TrajectoryFollower : public TrajectoryFollowerROS
{
public:
  TrajectoryFollower();

private:
  roscopter_msgs::msg::ControllerCommand output_cmd_;
  double dt_;
  bool params_initialized_;
  double max_accel_xy_;
  double max_accel_z_;

  roscopter::SimplePID PID_u_n_;
  roscopter::SimplePID PID_u_e_;
  roscopter::SimplePID PID_u_d_;
  
  // Functions
  void declare_params();
  double wrap_within_180(double datum, double angle_to_wrap);
  double north_control(double pn_cmd, double pn_dot_cmd, double pn_ddot_cmd);
  double east_control(double pe_cmd, double pe_dot_cmd, double pe_ddot_cmd);
  double down_control(double pd_cmd, double pd_dot_cmd, double pd_ddot_cmd);

  void update_gains();
  roscopter_msgs::msg::ControllerCommand manage_trajectory(roscopter_msgs::msg::TrajectoryCommand input_cmd, double dt);
  
};

}  // namespace roscopter


#endif