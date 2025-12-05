#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "roscopter_msgs/msg/waypoint.hpp"
#include "rviz_waypoint_publisher.hpp"

namespace roscopter_gcs
{

RvizWaypointPublisher::RvizWaypointPublisher()
    : Node("rviz_waypoint_publisher")
{
  declare_parameters();

  rclcpp::QoS qos_transient_local_20_(20);
  qos_transient_local_20_.transient_local();
  rviz_wp_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("rviz/waypoint",
                                                                         qos_transient_local_20_);
  waypoint_sub_ = this->create_subscription<roscopter_msgs::msg::Waypoint>(
    "waypoints", qos_transient_local_20_,
    std::bind(&RvizWaypointPublisher::new_wp_callback, this, std::placeholders::_1));

  num_wps_ = 0;
}

void RvizWaypointPublisher::declare_parameters()
{
  this->declare_parameter("waypoint_scale", 1.5);
  this->declare_parameter("text_scale", 5.0);
}

void RvizWaypointPublisher::new_wp_callback(const roscopter_msgs::msg::Waypoint & wp)
{
  if (wp.clear_wp_list) {
    publish_markers_to_clear_waypoints();
    line_points_.clear();
    num_wps_ = 0;
    return;
  }

  visualization_msgs::msg::Marker new_marker = create_new_waypoint_marker(wp);
  update_waypoint_line_list(wp);
  visualization_msgs::msg::Marker new_text = create_new_waypoint_text_marker(wp);

  rviz_wp_pub_->publish(new_marker);
  rviz_wp_pub_->publish(line_list_);
  rviz_wp_pub_->publish(new_text);

  ++num_wps_;
}

void RvizWaypointPublisher::publish_markers_to_clear_waypoints()
{
  visualization_msgs::msg::Marker new_marker;
  new_marker.header.stamp = this->get_clock()->now();
  new_marker.header.frame_id = "NED";
  new_marker.id = 0;
  new_marker.action = visualization_msgs::msg::Marker::DELETEALL;

  new_marker.ns = "wp";
  rviz_wp_pub_->publish(new_marker);
  new_marker.ns = "text";
  rviz_wp_pub_->publish(new_marker);
  new_marker.ns = "wp_path";
  rviz_wp_pub_->publish(new_marker);
}

visualization_msgs::msg::Marker RvizWaypointPublisher::create_new_waypoint_marker(const roscopter_msgs::msg::Waypoint& wp)
{
  visualization_msgs::msg::Marker new_marker;
  new_marker.header.stamp = this->get_clock()->now();
  new_marker.header.frame_id = "NED";
  new_marker.ns = "wp";
  new_marker.id = num_wps_;
  new_marker.type = visualization_msgs::msg::Marker::SPHERE;
  new_marker.action = visualization_msgs::msg::Marker::ADD;
  new_marker.pose.position.x = wp.w[0];
  new_marker.pose.position.y = wp.w[1];
  new_marker.pose.position.z = wp.w[2];
  new_marker.scale.x = this->get_parameter("waypoint_scale").as_double();
  new_marker.scale.y = this->get_parameter("waypoint_scale").as_double();
  new_marker.scale.z = this->get_parameter("waypoint_scale").as_double();
  new_marker.color.r = 1.0f;
  new_marker.color.g = 0.0f;
  new_marker.color.b = 0.0f;
  new_marker.color.a = 1.0;

  return new_marker;
}

void RvizWaypointPublisher::update_waypoint_line_list(const roscopter_msgs::msg::Waypoint& wp)
{
  geometry_msgs::msg::Point new_p;
  new_p.x = wp.w[0];
  new_p.y = wp.w[1];
  new_p.z = wp.w[2];
  line_points_.push_back(new_p);

  line_list_.header.stamp = this->get_clock()->now();
  line_list_.header.frame_id = "NED";
  line_list_.ns = "wp_path";
  line_list_.id = 0;
  line_list_.type = visualization_msgs::msg::Marker::LINE_STRIP;
  line_list_.action = visualization_msgs::msg::Marker::ADD;
  line_list_.scale.x = 1.0;
  line_list_.color.r = 0.0f;
  line_list_.color.g = 1.0f;
  line_list_.color.b = 0.0f;
  line_list_.color.a = 1.0;
  line_list_.points = line_points_;
}

visualization_msgs::msg::Marker RvizWaypointPublisher::create_new_waypoint_text_marker(const roscopter_msgs::msg::Waypoint& wp)
{
  visualization_msgs::msg::Marker new_text;
  new_text.header.stamp = this->get_clock()->now();
  new_text.header.frame_id = "NED";
  new_text.ns = "text";
  new_text.id = num_wps_;
  new_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
  new_text.action = visualization_msgs::msg::Marker::ADD;
  new_text.pose.position.x = wp.w[0];
  new_text.pose.position.y = wp.w[1];
  new_text.pose.position.z = wp.w[2] - this->get_parameter("waypoint_scale").as_double() - 1.0;
  new_text.scale.z = this->get_parameter("text_scale").as_double();
  new_text.color.r = 0.0f;
  new_text.color.g = 0.0f;
  new_text.color.b = 0.0f;
  new_text.color.a = 1.0;
  new_text.text = std::to_string(num_wps_);

  return new_text;
}

}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<roscopter_gcs::RvizWaypointPublisher>();

  rclcpp::spin(node);

  return 0;
}
