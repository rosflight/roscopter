#include <rclcpp/rclcpp.hpp>
#include "ekf/ekf_rclcpp.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<EKF_ROS::EKF_ROS>();

  rclcpp::spin(node);

  return 0;
}

