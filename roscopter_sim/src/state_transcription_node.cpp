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

    Eigen::Vector3f euler;
    euler(0) = atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                     pow(q.w(), 2) + pow(q.z(), 2) - pow(q.x(), 2) - pow(q.y(), 2));
    euler(1) = asin(2.0 * (q.w() * q.y() - q.x() * q.z()));
    euler(2) = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
                     pow(q.w(), 2) + pow(q.x(), 2) - pow(q.y(), 2) - pow(q.z(), 2));

    state.phi = euler(0);
    state.theta = euler(1);
    state.psi = euler(2);

    double u = msg.twist.twist.linear.x;
    double v = msg.twist.twist.linear.y;
    double w = msg.twist.twist.linear.z;

    state.v_n = u;
    state.v_e = v;
    state.v_d = w;

    state.vg = std::sqrt(pow(u, 2) + pow(v, 2) + pow(w, 2));

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