#include <controller/controller_state_machine.hpp>

using std::placeholders::_1;

namespace roscopter
{

ControllerStateMachine::ControllerStateMachine() : ControllerROS(), state_transition_(false), state_(DISARM), do_land_(false)
{
  declare_params();
  params.set_parameters();
}

void ControllerStateMachine::declare_params()
{
  params.declare_double("takeoff_d_pos", -10.0);
  params.declare_double("takeoff_height_threshold", 1.0);
  params.declare_double("takeoff_d_vel", -0.5);
  params.declare_double("takeoff_landing_pos_hold_time", 3.0);
}

void ControllerStateMachine::update_gains() {
  // No gains in the state machine need to be updated when parameters are changed. Do nothing.
}

rosflight_msgs::msg::Command ControllerStateMachine::manage_state(roscopter_msgs::msg::ControllerCommand & input_cmd, rosflight_msgs::msg::Status & status_msg, double dt)
{
  rosflight_msgs::msg::Command output_command;

  // Make sure dt is over a threshold so the PID loops don't blow up
  if(dt <= 0.0000001) {
    RCLCPP_WARN_STREAM(this->get_logger(), "dt <= 0.0000001");
    return output_command;
  }

  // Calculate the output command based on the state machine
  // TODO: disarm state is detriggered if input_cmd has valid command. This currently won't ever get reset to false after receiving a valid one. Unless high level controller sends it.
  // TODO: Implement a timer, etc. to reset it after a certain time?
  switch (state_) {
    case DISARM:
      manage_disarm(status_msg.armed, input_cmd.cmd_valid);
      break;

    case TAKEOFF:
      RCLCPP_INFO_STREAM_EXPRESSION(this->get_logger(), state_transition_, "TAKEOFF mode");
      state_transition_ = false;

      output_command = manage_takeoff(dt);
      break;

    case OFFBOARD:
      RCLCPP_WARN_STREAM_EXPRESSION(this->get_logger(), state_transition_, "OFFBOARD CONTROLLER ACTIVE");
      state_transition_ = false;

      output_command = compute_offboard_control(input_cmd, dt);

      if (!input_cmd.cmd_valid) {
        state_ = POSITION_HOLD;
      }
      break;

    case POSITION_HOLD:
      RCLCPP_INFO_STREAM_EXPRESSION(this->get_logger(), state_transition_, "POSITION_HOLD mode");
      state_transition_ = false;

      output_command = manage_position_hold(dt);
      break;

      // TODO: Currently nothing is available to have the vehicle land. It never will reach this state.
    case LANDING:
      RCLCPP_INFO_STREAM_EXPRESSION(this->get_logger(), state_transition_, "LANDING mode");
      state_transition_ = false;

      output_command = manage_landing();
      break;

    default:
      state_ = DISARM;
      break;
  }

  // Check to see if the disarmed
  if (!status_msg.armed) {
    state_ = DISARM;
  }

  // Check to see if control is valid
  // TODO: This will cause the state machine to reset after receiving invalid input commands... Figure out how to better handle this.
  // if (!input_cmd.cmd_valid && state_ != POSITION_HOLD) {
  //   state_ = POSITION_HOLD;
  //   RCLCPP_WARN_STREAM(this->get_logger(), "Entering POSITION_HOLD due to invalid input commands.");
  // }

  return output_command;
}

void ControllerStateMachine::manage_disarm(bool armed, bool cmd_valid)
{
  if (armed && cmd_valid) {
    // Change the state appropriately and set the takeoff positions
    state_ = TAKEOFF;
    takeoff_n_pos_ = xhat_.position[0];
    takeoff_e_pos_ = xhat_.position[1];
    takeoff_yaw_ = xhat_.psi;
  }
  else {
    RCLCPP_WARN_STREAM_EXPRESSION(this->get_logger(), !state_transition_, 
      "OFFBOARD CONTROLLER INACTIVE");
    state_transition_ = true;

    reset_integrators();
  }
}

rosflight_msgs::msg::Command ControllerStateMachine::manage_takeoff(double dt)
{
  double takeoff_d_pos = params.get_double("takeoff_d_pos");
  double takeoff_height_threshold = params.get_double("takeoff_height_threshold");
  double takeoff_d_vel = params.get_double("takeoff_d_vel");

  rosflight_msgs::msg::Command output_cmd;
  bool takeoff_complete = false;

  // Create high_level_command
  roscopter_msgs::msg::ControllerCommand input_cmd;
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DVEL_YAW;
  input_cmd.cmd1 = takeoff_n_pos_;
  input_cmd.cmd2 = takeoff_e_pos_;
  input_cmd.cmd3 = takeoff_d_vel;
  input_cmd.cmd4 = takeoff_yaw_;

  // Compute command
  output_cmd = compute_offboard_control(input_cmd, dt);

  // Check to see if the vehicle has attained the takeoff height (within the threshold)
  if (abs(xhat_.position[2] - takeoff_d_pos) <= takeoff_height_threshold) {
    takeoff_complete = true;
  }

  // Change state
  if (takeoff_complete) {
    state_ = POSITION_HOLD;
    start_position_hold_time_ = xhat_.header.stamp.sec + xhat_.header.stamp.nanosec * 1e-9;
    state_transition_ = true;
  }
  return output_cmd;
}

rosflight_msgs::msg::Command ControllerStateMachine::manage_position_hold(double dt)
{
  double takeoff_d_pos = params.get_double("takeoff_d_pos");
  double takeoff_landing_pos_hold_time = params.get_double("takeoff_landing_pos_hold_time");

  rosflight_msgs::msg::Command output_cmd;
  bool start_landing = false;
  bool start_offboard = false;

  // Create high level command
  roscopter_msgs::msg::ControllerCommand input_cmd;
  input_cmd.mode = roscopter_msgs::msg::ControllerCommand::MODE_NPOS_EPOS_DPOS_YAW;
  input_cmd.cmd1 = takeoff_n_pos_;
  input_cmd.cmd2 = takeoff_e_pos_;
  input_cmd.cmd3 = takeoff_d_pos;
  input_cmd.cmd4 = takeoff_yaw_;

  // Compute command
  output_cmd = compute_offboard_control(input_cmd, dt);

  // Check to see if the hold time is up
  double now = xhat_.header.stamp.sec + xhat_.header.stamp.nanosec * 1e-9;
  double elapsed_time = now - start_position_hold_time_;
  if (elapsed_time >= takeoff_landing_pos_hold_time) {
    if (do_land_) {
      start_landing = true;
    }
    else { start_offboard = true; }
  }

  // Transition states, as appropriate
  if (start_landing) {
    state_ = LANDING;
    state_transition_ = true;
  }
  else if (start_offboard) {
    state_ = OFFBOARD;
    state_transition_ = true;
  }

  return output_cmd;
}

rosflight_msgs::msg::Command ControllerStateMachine::manage_landing()
{
  rosflight_msgs::msg::Command output_cmd;
  bool landing_complete = false;

  if (landing_complete) {
    state_ = DISARM;
    state_transition_ = true;
  }
  return output_cmd;
}

}  // namespace controller
