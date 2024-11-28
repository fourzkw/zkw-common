#include "message_interfaces/msg/trackerparam.hpp"
#include "rclcpp/rclcpp.hpp"

class Tracker : public rclcpp::Node {
private:
  rclcpp::Subscription<message_interfaces::msg::Trackerparam>::SharedPtr
      command_subscribe_;

  void command_callback(message_interfaces::msg::Trackerparam::SharedPtr msg) {
    float ekf_xyz = msg->ekf_xyz;
    float ekf_yaw = msg->ekf_yaw;
    float ekf_r = msg->ekf_r;
    int vx = msg->vx;
    int vy = msg->vy;
    int vz = msg->vz;
    RCLCPP_INFO(this->get_logger(),
                "xyz=%.3f, yaw=%.3f, r=%.3f, vx=%d, vy=%d, vz=%d", ekf_xyz,
                ekf_yaw, ekf_r, vx, vy, vz);
  }

public:
  Tracker(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "%s节点已启动", name.c_str());

    command_subscribe_ =
        this->create_subscription<message_interfaces::msg::Trackerparam>(
            "command", 10,
            std::bind(&Tracker::command_callback, this, std::placeholders::_1));
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Tracker>("tracker_node");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}