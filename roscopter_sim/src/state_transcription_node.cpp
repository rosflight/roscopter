#include <memory>

#include "Eigen/Geometry"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roscopter_msgs/msg/state.hpp"

#include "chrono"

using namespace std::chrono_literals;
using roscopter_msgs::msg::State;
using std::placeholders::_1;

class gazebo_transcription : public rclcpp::Node
{
public:
  gazebo_transcription()
      : Node("gazebo_state")
  {
    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/multirotor/truth/NED", 10, std::bind(&gazebo_transcription::publish_truth, this, _1));

    publisher_ = this->create_publisher<roscopter_msgs::msg::State>("state", 10);
  }

private:
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr
    euler_sub_; // TODO is there going to be gitter between the times each of these collected?

  rclcpp::Publisher<State>::SharedPtr publisher_;

  //TODO insert wind callback.
  double wn = 0.0;
  double we = 0.0;
  double wd = 0.0;

  void publish_truth(const nav_msgs::msg::Odometry & msg)
  {

    roscopter_msgs::msg::State state;

    state.header.stamp = this->get_clock()->now();
    state.header.frame_id = 1; // Denotes global frame.

    state.initial_lat = 0.0;
    state.initial_lon = 0.0; // TODO implement correct initial lat and lon
    state.initial_alt = 0.0;

    state.position[0] = msg.pose.pose.position.x;
    state.position[1] = msg.pose.pose.position.y;
    state.position[2] = msg.pose.pose.position.z;

    Eigen::Quaternionf q;
    q.w() = msg.pose.pose.orientation.w;
    q.x() = msg.pose.pose.orientation.x;
    q.y() = msg.pose.pose.orientation.y;
    q.z() = msg.pose.pose.orientation.z;

    state.inclination = 0.0;

    Eigen::Vector3f euler;
    euler(0) = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                     pow(q.w(), 2) + pow(q.z(), 2) - pow(q.x(), 2) - pow(q.y(), 2));
    euler(1) = asin(2.0 * (q.w() * q.y() - q.x() * q.z()));
    euler(2) = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     pow(q.w(), 2) + pow(q.x(), 2) - pow(q.y(), 2) - pow(q.z(), 2));

    state.phi = euler(0);
    state.theta = euler(1);
    state.psi = euler(2);

    Eigen::Vector3f body_frame_velocity;
    body_frame_velocity << msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z;

    // Rotate the velocity vector from the body frame to the inertial frame.
    Eigen::Matrix3f Rv_v1, Rv1_v2, Rv2_b;
    Rv2_b << 1, 0, 0, 0, cos(state.phi), sin(state.phi), 0, -sin(state.phi), cos(state.phi);
    Rv1_v2 << cos(state.theta), 0, -sin(state.theta), 0, 1, 0, sin(state.theta), 0, cos(state.theta);
    Rv_v1 << cos(state.psi), sin(state.psi), 0, -sin(state.psi), cos(state.psi), 0, 0, 0, 1;

    Eigen::Matrix3f Rb_i = (Rv2_b * Rv1_v2 * Rv_v1).transpose();

    Eigen::Vector3f inertial_frame_velocity = Rb_i * body_frame_velocity;

    state.v_n = inertial_frame_velocity(0);
    state.v_e = inertial_frame_velocity(1);
    state.v_d = inertial_frame_velocity(2);

    state.vg = std::sqrt(pow(inertial_frame_velocity(0), 2)
		       + pow(inertial_frame_velocity(1), 2)
		       + pow(inertial_frame_velocity(2), 2));

    state.p = msg.twist.twist.angular.x;
    state.q = msg.twist.twist.angular.y;
    state.r = msg.twist.twist.angular.z;

    state.quat_valid = true;

    state.quat[0] = msg.pose.pose.orientation.w;
    state.quat[1] = msg.pose.pose.orientation.x;
    state.quat[2] = msg.pose.pose.orientation.y;
    state.quat[3] = msg.pose.pose.orientation.z;

    publisher_->publish(state);
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gazebo_transcription>());
  rclcpp::shutdown();
  return 0;
}
