#ifndef TRAJECTORY_FOLLOWER_HPP
#define TRAJECTORY_FOLLOWER_HPP

#include <stdint.h>

#include <Eigen/Geometry>

#include "navigation/trajectory_follower_ros.hpp"
#include "controller/simple_pid.hpp"

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

  roscopter::SimplePID PID_u_n_;
  roscopter::SimplePID PID_u_e_;
  roscopter::SimplePID PID_u_d_;
  roscopter::SimplePID PID_yaw_to_rate_;

  // Functions
  void declare_params();
  Eigen::Vector4d invert_control_inputs(const Eigen::Vector4d u);
  double compute_theta_dot(const Eigen::Vector3d z, double thrust, const Eigen::Vector4d u);
  Eigen::Matrix3d R_psi(double psi);
  double wrap_within_180(double datum, double angle_to_wrap);
  Eigen::Vector4d compute_control_input(const double pn_cmd, const double pe_cmd, const double pd_cmd, const double psi_cmd,
                                        const double vn, const double ve, const double vd);
  double north_control(const double pn_cmd, const double vn);
  double east_control(const double pe_cmd, const double ve);
  double down_control(const double pd_cmd, const double vd);
  double psi_control(const double psi_cmd);

  void update_gains() override;
  roscopter_msgs::msg::ControllerCommand manage_trajectory(roscopter_msgs::msg::TrajectoryCommand input_cmd, double dt) override;
  void clear_integrators() override;
};

}  // namespace roscopter


#endif
