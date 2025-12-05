#ifndef RVIZ_WAYPOINT_PUBLISHER_HPP
#define RVIZ_WAYPOINT_PUBLISHER_HPP

#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "roscopter_msgs/msg/state.hpp"
#include "roscopter_msgs/msg/waypoint.hpp"

namespace roscopter_gcs
{

class RvizWaypointPublisher : public rclcpp::Node
{
public:
  RvizWaypointPublisher();

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_wp_pub_;
  rclcpp::Subscription<roscopter_msgs::msg::Waypoint>::SharedPtr waypoint_sub_;

  void declare_parameters();

  void new_wp_callback(const roscopter_msgs::msg::Waypoint & wp);
  void publish_markers_to_clear_waypoints();
  visualization_msgs::msg::Marker create_new_waypoint_marker(const roscopter_msgs::msg::Waypoint& wp);
  void update_waypoint_line_list(const roscopter_msgs::msg::Waypoint& wp);
  visualization_msgs::msg::Marker create_new_waypoint_text_marker(const roscopter_msgs::msg::Waypoint& wp);

  // Persistent rviz markers
  visualization_msgs::msg::Marker line_list_;
  std::vector<geometry_msgs::msg::Point> line_points_;

  int num_wps_;
};

} // namespace roscopter_gcs

#endif
