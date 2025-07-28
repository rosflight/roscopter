#include <memory>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

#include "roscopter_msgs/msg/state.hpp"
#include "rosflight_msgs/msg/sim_state.hpp"

namespace roscopter_sim {

class SimStateTranscription : public rclcpp::Node
{
public:
  SimStateTranscription()
      : Node("roscopter_state_transcription")
  {
    sim_state_sub_ = this->create_subscription<rosflight_msgs::msg::SimState>(
      "sim/truth_state", 10, std::bind(&SimStateTranscription::publish_truth, this, std::placeholders::_1));
    wind_truth_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "sim/wind_truth", 10, std::bind(&SimStateTranscription::wind_callback, this, std::placeholders::_1));

    roscopter_state_pub_ = this->create_publisher<roscopter_msgs::msg::State>("sim/roscopter/state", 10);
  }

private:
  rclcpp::Subscription<rosflight_msgs::msg::SimState>::SharedPtr sim_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr wind_truth_sub_;

  rclcpp::Publisher<roscopter_msgs::msg::State>::SharedPtr roscopter_state_pub_;

  double wn_ = 0.0;
  double we_ = 0.0;
  double wd_ = 0.0;

  void wind_callback(const geometry_msgs::msg::Vector3Stamped & msg)
  {
    // Wind velocities in the inertial frame
    wn_ = msg.vector.x;
    we_ = msg.vector.y;
    wd_ = msg.vector.z;
  }

  void publish_truth(const rosflight_msgs::msg::SimState & msg)
  {
    roscopter_msgs::msg::State state;

    // TODO: Should this use the timestamp of the incoming estimate?
    state.header.stamp = this->get_clock()->now();
    state.header.frame_id = 1; // Denotes global frame.

    state.initial_lat = 0.0;
    state.initial_lon = 0.0; // TODO: implement correct initial lat and lon
    state.initial_alt = 0.0;

    // Inertial NED frame
    state.position[0] = msg.pose.position.x;
    state.position[1] = msg.pose.position.y;
    state.position[2] = msg.pose.position.z;

    // Quaternion is from body to inertial
    Eigen::Quaternionf q;
    q.w() = msg.pose.orientation.w;
    q.x() = msg.pose.orientation.x;
    q.y() = msg.pose.orientation.y;
    q.z() = msg.pose.orientation.z;

    // Equation B.1 in Small Unmanned Aircraft
    state.phi = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                     pow(q.w(), 2) + pow(q.z(), 2) - pow(q.x(), 2) - pow(q.y(), 2));
    state.theta = asin(2.0 * (q.w() * q.y() - q.x() * q.z()));
    state.psi = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     pow(q.w(), 2) + pow(q.x(), 2) - pow(q.y(), 2) - pow(q.z(), 2));

    // Inertial linear velocities in body frame
    Eigen::Vector3f body_frame_velocity(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
    state.v_x = body_frame_velocity[0];
    state.v_y = body_frame_velocity[1];
    state.v_z = body_frame_velocity[2];

    // Angular velocities in body frame
    state.p = msg.twist.angular.x;
    state.q = msg.twist.angular.y;
    state.r = msg.twist.angular.z;

    // TODO: Add the true biases to the state publisher or to a standalone_sensor publisher

    state.quat_valid = true;

    state.quat[0] = msg.pose.orientation.w;
    state.quat[1] = msg.pose.orientation.x;
    state.quat[2] = msg.pose.orientation.y;
    state.quat[3] = msg.pose.orientation.z;

    roscopter_state_pub_->publish(state);
  }
};

} // namespace roscopter_sim

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<roscopter_sim::SimStateTranscription>());
  rclcpp::shutdown();
  return 0;
}
