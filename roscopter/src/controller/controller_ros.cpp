#include <controller/controller_ros.hpp>
#include <controller/controller.hpp>
#include <controller/controller_successive_loop.hpp>

using std::placeholders::_1;

namespace roscopter
{

ControllerROS::ControllerROS() : Node("controller"), params(this)
{
  is_flying_.data = false;
  received_cmd_ = false;
  first_active_control_loop_ = true;

  // Set up Publisher and Subscribers
  state_sub_ = this->create_subscription<roscopter_msgs::msg::State>("estimated_state", 1, std::bind(&ControllerROS::state_callback, this, _1));
  is_flying_sub_ = this->create_subscription<roscopter_msgs::msg::Bool>("is_flying", 1, std::bind(&ControllerROS::is_flying_callback, this, _1));
  cmd_sub_ = this->create_subscription<roscopter_msgs::msg::ControllerCommand>("high_level_command", 1, std::bind(&ControllerROS::cmd_callback, this, _1));
  status_sub_ = this->create_subscription<rosflight_msgs::msg::Status>("status", 1, std::bind(&ControllerROS::status_callback, this, _1));
  command_pub_ = this->create_publisher<rosflight_msgs::msg::Command>("command", 1);

  // Declare params with ROS and the ParamManager
  declare_params();
  params.set_parameters();

  // TODO: Does the firmware do anythign with this parameter? i.e., does it get sent to the firmware? It would be better to remove it from here
  if (!params.get_double("equilibrium_throttle")) {
    RCLCPP_ERROR(this->get_logger(), "Controller MAV equilibrium throttle not found!");
  }
  // Register parameter callback
  parameter_callback_handle_ = this->add_on_set_parameters_callback(std::bind(&ControllerROS::parameters_callback, this, _1));
}

rcl_interfaces::msg::SetParametersResult ControllerROS::parameters_callback(const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = false;
  result.reason = "One of the parameters is not a parameter of the controller.";

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

void ControllerROS::declare_params()
{
  params.declare_double("equilibrium_throttle", 0.5); // Default values
  params.declare_double("max_roll", 0.15);  
  params.declare_double("max_pitch", 0.15);
  params.declare_double("max_yaw_rate", 0.15);
  params.declare_double("max_throttle", 0.85);
  params.declare_double("max_n_dot", 0.15);
  params.declare_double("max_e_dot", 0.15);
  params.declare_double("max_d_dot", 0.15);

  params.declare_double("min_altitude", 0.15);

  params.declare_double("x_dot_P", 0.5);
  params.declare_double("x_dot_I", 0.0);
  params.declare_double("x_dot_D", 0.05);

  params.declare_double("y_dot_P", 0.5);
  params.declare_double("y_dot_I", 0.0);
  params.declare_double("y_dot_D", 0.05);

  params.declare_double("z_dot_P", 0.4);
  params.declare_double("z_dot_I", 0.25);
  params.declare_double("z_dot_D", 0.1);

  params.declare_double("north_P", 1.0);
  params.declare_double("north_I", 0.1);
  params.declare_double("north_D", 0.35);

  params.declare_double("east_P", 1.0);
  params.declare_double("east_I", 0.1);
  params.declare_double("east_D", 0.2);

  params.declare_double("down_P", 1.0);
  params.declare_double("down_I", 0.0);
  params.declare_double("down_D", 0.0);

  params.declare_double("psi_P", 2.0);
  params.declare_double("psi_I", 0.0);
  params.declare_double("psi_D", 0.0);

  params.declare_double("tau", 0.05);
}

void ControllerROS::cmd_callback(const roscopter_msgs::msg::ControllerCommand &msg)
{
  input_cmd_ = msg;
  // switch(msg.mode)
  // {
  //   case roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DPOS_YAW:
  //     xc_.pn = msg.cmd1;
  //     xc_.pe = msg.cmd2;
  //     xc_.pd = msg.cmd3;
  //     xc_.psi = msg.cmd4;
  //     control_mode_ = msg.mode;
  //     break;
  //   case roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DVEL_YAW:
  //     xc_.pn = msg.cmd1;
  //     xc_.pe = msg.cmd2;
  //     xc_.z_dot = msg.cmd3;
  //     xc_.psi = msg.cmd4;
  //     control_mode_ = msg.mode;
  //     break;
  //   // case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_ALTITUDE:
  //   case roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DPOS_YAWRATE:
  //     xc_.x_dot = msg.cmd1;
  //     xc_.y_dot = msg.cmd2;
  //     xc_.pd = msg.cmd3;
  //     xc_.r = msg.cmd4;
  //     control_mode_ = msg.mode;
  //     break;
  //   // case rosflight_msgs::Command::MODE_XVEL_YVEL_YAWRATE_Z_VEL:
  //   case roscopter_msgs::msg::ControllerCommand::MODE_NVEL_EVEL_DVEL_YAWRATE:
  //     xc_.x_dot = msg.cmd1;
  //     xc_.y_dot = msg.cmd2;
  //     xc_.z_dot = msg.cmd3;
  //     xc_.r = msg.cmd4;
  //     control_mode_ = msg.mode;
  //     break;
  //   // case rosflight_msgs::Command::MODE_XACC_YACC_YAWRATE_AZ:
  //   case roscopter_msgs::msg::ControllerCommand::MODE_NACC_EACC_DACC_YAWRATE:
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
  if(prev_time == 0) {
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

  // Save the estimated state for use in derived classes
  xhat_ = msg;

  if(is_flying_.data && status_.armed && received_cmd_)
  {
    RCLCPP_WARN_STREAM_EXPRESSION(this->get_logger(), first_active_control_loop_, "CONTROLLER ACTIVE");
    first_active_control_loop_ = false;

    // Compute control commands
    rosflight_msgs::msg::Command command = compute_control(input_cmd_, dt);

    // Publish command to the firmware
    publish_command(command);
  }
  // TODO : Replace the reset_integrators() witha a call in the inherited class. i.e., don't do it here
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

  if (strcmp(argv[1], "default") == 0) {
    auto node = std::make_shared<roscopter::ControllerSuccessiveLoop>();
    RCLCPP_INFO_ONCE(node->get_logger(), "Using default (cascading PID) controller");
    rclcpp::spin(node);
  } else if (strcmp(argv[1], "other") == 0) {
    auto node = std::make_shared<roscopter::Controller>();
    RCLCPP_INFO_ONCE(node->get_logger(), "Using other controller");
    rclcpp::spin(node);
  }

  return 0;
}
