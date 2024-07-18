#include <controller/controller_ros.hpp>
#include <controller/controller.hpp>
#include <controller/controller_successive_loop.hpp>

using std::placeholders::_1;

namespace roscopter
{

ControllerROS::ControllerROS() : Node("controller")
{
  is_flying_.data = false;
  received_cmd_ = false;
  first_active_control_loop_ = true;

  // Declare params with ROS and the ParamManager
  declare_params();

  if (!this->get_parameter("equilibrium_throttle").as_double()) {
    RCLCPP_ERROR(this->get_logger(), "Controller MAV equilibrium throttle not found!");
  }

  // Set up Publisher and Subscribers
  state_sub_ = this->create_subscription<roscopter_msgs::msg::State>("estimated_state", 1, std::bind(&ControllerROS::state_callback, this, _1));
  is_flying_sub_ = this->create_subscription<roscopter_msgs::msg::Bool>("is_flying", 1, std::bind(&ControllerROS::is_flying_callback, this, _1));
  cmd_sub_ = this->create_subscription<roscopter_msgs::msg::Command>("high_level_command", 1, std::bind(&ControllerROS::cmd_callback, this, _1));
  status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>("status", 1, std::bind(&ControllerROS::status_callback, this, _1));
  
  command_pub_ = this->create_publisher<rosflight_msgs::msg::Command>("command", 1);

  // Register parameter callback
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ControllerROS::parameters_callback, this, _1));

}

rcl_interfaces::msg::SetParametersResult ControllerROS::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "TODO: Implement parameter manager";

  for (auto param : parameters) {
    RCLCPP_INFO_STREAM(this->get_logger(), param.get_name() << " was not changed internally!");
  }

  return result;
} 

void ControllerROS::declare_params()
{
  this->declare_parameter("equilibrium_throttle", 0.5); // Default values
  this->declare_parameter("max_roll", 0.15);  
  this->declare_parameter("max_pitch", 0.15);
  this->declare_parameter("max_yaw_rate", 0.15);
  this->declare_parameter("max_throttle", 0.85);
  this->declare_parameter("max_n_dot", 0.15);
  this->declare_parameter("max_e_dot", 0.15);
  this->declare_parameter("max_d_dot", 0.15);

  this->declare_parameter("min_altitude", 0.15);

  this->declare_parameter("x_dot_P", 0.5);
  this->declare_parameter("x_dot_I", 0.0);
  this->declare_parameter("x_dot_D", 0.05);

  this->declare_parameter("y_dot_P", 0.5);
  this->declare_parameter("y_dot_I", 0.0);
  this->declare_parameter("y_dot_D", 0.05);

  this->declare_parameter("z_dot_P", 0.4);
  this->declare_parameter("z_dot_I", 0.25);
  this->declare_parameter("z_dot_D", 0.1);

  this->declare_parameter("north_P", 1.0);
  this->declare_parameter("north_I", 0.1);
  this->declare_parameter("north_D", 0.35);

  this->declare_parameter("east_P", 1.0);
  this->declare_parameter("east_I", 0.1);
  this->declare_parameter("east_D", 0.2);

  this->declare_parameter("down_P", 1.0);
  this->declare_parameter("down_I", 0.0);
  this->declare_parameter("down_D", 0.0);

  this->declare_parameter("psi_P", 2.0);
  this->declare_parameter("psi_I", 0.0);
  this->declare_parameter("psi_D", 0.0);

  this->declare_parameter("tau", 0.05);

}

void ControllerROS::cmd_callback(const roscopter_msgs::msg::Command &msg)
{
  input_cmd_ = msg;
  // switch(msg.mode)
  // {
  //   case roscopter_msgs::msg::Command::MODE_NPOS_EPOS_DPOS_YAW:
  //     xc_.pn = msg.cmd1;
  //     xc_.pe = msg.cmd2;
  //     xc_.pd = msg.cmd3;
  //     xc_.psi = msg.cmd4;
  //     control_mode_ = msg.mode;
  //     break;
  //   case roscopter_msgs::msg::Command::MODE_NPOS_EPOS_DVEL_YAW:
  //     xc_.pn = msg.cmd1;
  //     xc_.pe = msg.cmd2;
  //     xc_.z_dot = msg.cmd3;
  //     xc_.psi = msg.cmd4;
  //     control_mode_ = msg.mode;
  //     break;
  //   // case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
  //   case roscopter_msgs::msg::Command::MODE_NVEL_EVEL_DPOS_YAWRATE:
  //     xc_.x_dot = msg.cmd1;
  //     xc_.y_dot = msg.cmd2;
  //     xc_.pd = msg.cmd3;
  //     xc_.r = msg.cmd4;
  //     control_mode_ = msg.mode;
  //     break;
  //   // case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_Z_VEL:
  //   case roscopter_msgs::msg::Command::MODE_NVEL_EVEL_DVEL_YAWRATE:
  //     xc_.x_dot = msg.cmd1;
  //     xc_.y_dot = msg.cmd2;
  //     xc_.z_dot = msg.cmd3;
  //     xc_.r = msg.cmd4;
  //     control_mode_ = msg.mode;
  //     break;
  //   // case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
  //   case roscopter_msgs::msg::Command::MODE_NACC_EACC_DACC_YAWRATE:
  //     xc_.ax = msg.cmd1;
  //     xc_.ay = msg.cmd2;
  //     xc_.az = msg.cmd3;
  //     xc_.r = msg.cmd4;
  //     control_mode_ = msg.mode;
  //     break;
  //   default:
  //     RCLCPP_ERROR(this->get_logger(), "roscopter/controller: Unhandled command message of type %d",
  //               msg.mode);
  //     break;
  // }

  if (!received_cmd_)
    received_cmd_ = true;
}

void ControllerROS::state_callback(const roscopter_msgs::msg::State &msg)
{
  RCLCPP_INFO_ONCE(this->get_logger(), "Started receiving estimated state message.");

  static double prev_time = 0;
  if(prev_time == 0)
  {
    prev_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
    return;
  }

  // Calculate time
  double now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9;
  double dt = now - prev_time;
  prev_time = now;

  if(dt <= 0) {
    return;
  }

  roscopter_msgs::msg::State xhat = msg;

  if(is_flying_.data && status_.armed && received_cmd_)
  {
    RCLCPP_WARN_STREAM_EXPRESSION(this->get_logger(), first_active_control_loop_, "CONTROLLER ACTIVE");
    first_active_control_loop_ = false;

    rosflight_msgs::msg::Command command = compute_control(xhat, input_cmd_, dt);

    publish_command(command);
  }
  // TODO: Replace the reset_integrators() witha a call in the inherited class. i.e., don't do it here
  else {
    RCLCPP_WARN_STREAM_EXPRESSION(this->get_logger(), !first_active_control_loop_, "CONTROLLER INACTIVE");
    first_active_control_loop_ = true;

    reset_integrators();
  }
}


void ControllerROS::is_flying_callback(const roscopter_msgs::msg::Bool &msg)
{
  is_flying_ = msg;
}

void ControllerROS::status_callback(const rosflight_msgs::msg::Status &msg)
{
  status_ = msg;
}

void ControllerROS::publish_command(rosflight_msgs::msg::Command &command)
{
  command.header.stamp = this->get_clock()->now();
  command_pub_->publish(command);
}

double ControllerROS::saturate(double x, double max, double min)
{
  x = (x > max) ? max : x;
  x = (x < min) ? min : x;
  return x;
}


}  // namespace controller


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  if (strcmp(argv[1], "successive_loop") == 0) {
    auto node = std::make_shared<roscopter::ControllerSuccessiveLoop>();
    RCLCPP_INFO_ONCE(node->get_logger(), "Using successive loop controller");
    rclcpp::spin(node);
  } else if (strcmp(argv[1], "other") == 0) {
    auto node = std::make_shared<roscopter::Controller>();
    RCLCPP_INFO_ONCE(node->get_logger(), "Using other controller");
    rclcpp::spin(node);
  }

  return 0;
}
