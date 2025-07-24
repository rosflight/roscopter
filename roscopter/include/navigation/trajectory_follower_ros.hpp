#ifndef TRAJECTORY_FOLLOWER_ROS_H
#define TRAJECTORY_FOLLOWER_ROS_H

#include <stdint.h>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "param_manager/param_manager.hpp"
#include "roscopter_msgs/msg/controller_command.hpp"
#include "roscopter_msgs/msg/trajectory_command.hpp"
#include "roscopter_msgs/msg/state.hpp"

using std::placeholders::_1;

namespace roscopter
{

class TrajectoryFollowerROS : public rclcpp::Node
{

public:

  TrajectoryFollowerROS();

  static double saturate(double x, double max, double min);

protected:
  ParamManager params;  
  roscopter_msgs::msg::State xhat_;     /** Current estimated state of MAV */

private:
  bool received_cmd_msg_;

  // Publishers and Subscribers
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::TrajectoryCommand>::SharedPtr trajectory_sub_;
  rclcpp::Publisher<roscopter_msgs::msg::ControllerCommand>::SharedPtr cmd_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_integrators_srvs_;

  // Memory for sharing information between functions
  roscopter_msgs::msg::TrajectoryCommand input_cmd_;  /** High level, input trajectory commands to the trajectory follower */

  // Functions
  double compute_dt(double now);
  void state_callback(const roscopter_msgs::msg::State &msg);
  void cmd_callback(const roscopter_msgs::msg::TrajectoryCommand &msg);
  void publish_command(roscopter_msgs::msg::ControllerCommand &command);
  bool clear_integrators_callback(const std_srvs::srv::Trigger::Request::SharedPtr req, const std_srvs::srv::Trigger::Response::SharedPtr res);

  virtual void update_gains() = 0;
  virtual roscopter_msgs::msg::ControllerCommand manage_trajectory(roscopter_msgs::msg::TrajectoryCommand input_cmd, double dt) = 0;
  virtual void clear_integrators() = 0;

  /**
   * @brief Declares parameters with ROS2 and loads from a parameter file, if given
   */
  void declare_params();

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;

  /**
   * @brief Callback function to handle parameter changes
   * 
   * @param parameters: Vector of rclcpp::Parameter objects that were changed
   */
  rcl_interfaces::msg::SetParametersResult parameters_callback(const std::vector<rclcpp::Parameter> & parameters);

};
}

#endif
