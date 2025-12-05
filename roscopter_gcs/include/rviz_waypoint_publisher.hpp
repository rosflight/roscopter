#ifndef RVIZ_WAYPOINT_PUBLISHER_HPP
#define RVIZ_WAYPOINT_PUBLISHER_HPP

#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker.hpp>

#include "roscopter_msgs/msg/state.hpp"
#include "roscopter_msgs/msg/waypoint.hpp"

#define SCALE 1.5
#define TEXT_SCALE 5.0
#define PATH_PUBLISH_MOD 1
#define MAX_PATH_HISTORY 10000
using std::placeholders::_1;

namespace roscopter_gcs
{

class RvizWaypointPublisher : public rclcpp::Node
{
public:
  RvizWaypointPublisher();
  ~RvizWaypointPublisher();

private:
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_wp_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_mesh_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr rviz_aircraft_path_pub_;
  rclcpp::Subscription<roscopter_msgs::msg::Waypoint>::SharedPtr waypoint_sub_;
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr vehicle_state_sub_;

  std::unique_ptr<tf2_ros::TransformBroadcaster> aircraft_tf2_broadcaster_;

  void new_wp_callback(const roscopter_msgs::msg::Waypoint & wp);
  void state_update_callback(const roscopter_msgs::msg::State & state);
  void update_list();
  void update_mesh();
  void update_aircraft_history();

  void publish_markers_to_clear_waypoints();

  roscopter_msgs::msg::State vehicle_state_;

  // Persistent rviz markers
  visualization_msgs::msg::Marker line_list_;
  std::vector<geometry_msgs::msg::Point> line_points_;
  visualization_msgs::msg::Marker aircraft_;
  visualization_msgs::msg::Marker aircraft_history_;
  std::vector<geometry_msgs::msg::Point> aircraft_history_points_;

  int num_wps_;
  int i_;
};

} // namespace roscopter_gcs

#endif
