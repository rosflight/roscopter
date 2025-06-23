#include <rclcpp/rclcpp.hpp>

#include <roscopter_msgs/msg/state.hpp>
#include <rosflight_msgs/msg/attitude.hpp>

using std::placeholders::_1;

namespace roscopter
{

class ExternalAttitudeTranscriber : public rclcpp::Node
{
public:
  ExternalAttitudeTranscriber() : Node("ext_att_transcriber") {
    // Set up publishers and subscribers
    ext_att_pub_ = this->create_publisher<rosflight_msgs::msg::Attitude>("external_attitude", 1);
    state_sub_ = this->create_subscription<roscopter_msgs::msg::State>("estimated_state", 1, std::bind(&ExternalAttitudeTranscriber::state_sub_callback, this, _1));
  }

private:
  rclcpp::Publisher<rosflight_msgs::msg::Attitude>::SharedPtr ext_att_pub_;
  rclcpp::Subscription<roscopter_msgs::msg::State>::SharedPtr state_sub_;


  void state_sub_callback(const roscopter_msgs::msg::State & msg)
  {
    rosflight_msgs::msg::Attitude out_msg;
    out_msg.header.stamp = this->get_clock()->now();

    // Only publish external attitude information if the quaternion is valid
    if (msg.quat_valid) {
        out_msg.attitude.x = msg.quat[1];
        out_msg.attitude.y = msg.quat[2];
        out_msg.attitude.z = msg.quat[3];
        out_msg.attitude.w = msg.quat[0];

        out_msg.angular_velocity.x = msg.p;
        out_msg.angular_velocity.y = msg.q;
        out_msg.angular_velocity.z = msg.r;

        ext_att_pub_->publish(out_msg);
    }
  }

};

}   // namespace roscopter


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<roscopter::ExternalAttitudeTranscriber>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
