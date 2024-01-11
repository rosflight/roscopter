#include <rclcpp/rclcpp.hpp>
#include "controller/controller.h"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  // ros::NodeHandle nh;

  // controller::Controller Thing;
  auto node = std::make_shared<controller::Controller>();

  rclcpp::spin(node);

  return 0;
}
