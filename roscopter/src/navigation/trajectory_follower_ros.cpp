#include "navigation/trajectory_follower_ros.hpp"
#include "navigation/trajectory_follower.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

namespace roscopter
{

TrajectoryFollowerROS::TrajectoryFollowerROS()
  : Node("trajectory_follower")
  , params(this)
  , xhat_(roscopter_msgs::msg::State())
  , firmware_status_(rosflight_msgs::msg::Status())
  , received_cmd_msg_(false)
  , input_cmd_(roscopter_msgs::msg::TrajectoryCommand())
{
  // Instantiate publishers and subscribers
  state_sub_ = this->create_subscription<roscopter_msgs::msg::State>("estimated_state", 1, std::bind(&TrajectoryFollowerROS::state_callback, this, _1));
  trajectory_sub_ = this->create_subscription<roscopter_msgs::msg::TrajectoryCommand>("trajectory_command", 1, std::bind(&TrajectoryFollowerROS::cmd_callback, this, _1));
  firmware_status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>("status", 1, std::bind(&TrajectoryFollowerROS::status_callback, this, _1));
  cmd_pub_ = this->create_publisher<roscopter_msgs::msg::ControllerCommand>("high_level_command", 1);

  // Clear integrators service
  clear_integrators_srvs_ = this->create_service<std_srvs::srv::Trigger>(
    "trajectory_follower/clear_integrators", std::bind(&TrajectoryFollowerROS::clear_integrators_callback, this, _1, _2));

  // Register parameter callback
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&TrajectoryFollowerROS::parameters_callback, this, _1));
  
  declare_params();
  params.set_parameters();
}

void TrajectoryFollowerROS::declare_params()
{
  // Declare params here needed in this node
}

rcl_interfaces::msg::SetParametersResult TrajectoryFollowerROS::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters is not a parameter of the trajectory follower.";

  // Use the ParamManager's set parameters callback
  bool success = params.set_parameters_callback(parameters);
  if (success)
  {
    result.successful = true;
    result.reason = "success";
  }

  // Call the update_gains method implemented in a derived class to save the gains in member variables there.
  update_gains();

  return result;
}

void TrajectoryFollowerROS::state_callback(const roscopter_msgs::msg::State &msg)
{
  xhat_ = msg;

  // If the trajectory follower has not received an input command yet, do not compute control commands or publish
  if (!received_cmd_msg_) { return; }

  // Calculate dt and return if dt is invalid
  double now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  double dt = compute_dt(now);

  roscopter_msgs::msg::ControllerCommand output_cmd = manage_trajectory(input_cmd_, dt);

  publish_command(output_cmd);
}

double TrajectoryFollowerROS::compute_dt(double now)
{
  static double prev_time = 0;
  if(prev_time == 0) {
    prev_time = now;
    // Don't compute control since we don't have a dt calculation
    return 0;
  }

  // Calculate time
  double dt = now - prev_time;
  prev_time = now;

  return dt;
}

void TrajectoryFollowerROS::cmd_callback(const roscopter_msgs::msg::TrajectoryCommand &msg)
{
  input_cmd_ = msg;
  received_cmd_msg_ = true;
}

void TrajectoryFollowerROS::status_callback(const rosflight_msgs::msg::Status &msg)
{
  firmware_status_ = msg;
}

void TrajectoryFollowerROS::publish_command(roscopter_msgs::msg::ControllerCommand &command)
{
  command.header.stamp = this->get_clock()->now();
  cmd_pub_->publish(command);
}

bool TrajectoryFollowerROS::clear_integrators_callback(const std_srvs::srv::Trigger::Request::SharedPtr req,
                                                       const std_srvs::srv::Trigger::Response::SharedPtr res)
{
  clear_integrators();
  res->success = true;
  return true;
}

double TrajectoryFollowerROS::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}

}   // namespace roscopter


int main(int argc, char* argv[])
{
    // Initialize the ROS2 client library
    rclcpp::init(argc, argv);

    // Create and spin node
    auto node = std::make_shared<roscopter::TrajectoryFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
