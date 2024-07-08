#include <rclcpp/rclcpp.hpp>
#include "ekf/ekf_ros.h"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<roscopter::ekf::EKF_ROS>();

  rclcpp::spin(node);

  return 0;
}

