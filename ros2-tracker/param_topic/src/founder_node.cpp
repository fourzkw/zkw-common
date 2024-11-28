#include "message_interfaces/msg/trackerparam.hpp"
#include "rclcpp/rclcpp.hpp"

class Founder : public rclcpp::Node {
private:
  //数据
  float ekf_xyz, ekf_yaw, ekf_r;
  int vx, vy, vz;

  //声明话题发布者
  rclcpp::Publisher<message_interfaces::msg::Trackerparam>::SharedPtr command_publisher_;
  //声明定时器
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback();

  void init_param();

public:
  Founder(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "%s节点已启动.", name.c_str());
    init_param();
    command_publisher_ =
        this->create_publisher<message_interfaces::msg::Trackerparam>("command", 10);

    timer_ = create_wall_timer(std::chrono::seconds(1),
                      std::bind(&Founder::timer_callback, this));
  }
};

void Founder::timer_callback() {
  message_interfaces::msg::Trackerparam message;
  message.ekf_xyz = ekf_xyz;
  message.ekf_yaw = ekf_yaw;
  message.ekf_r = ekf_r;
  message.vx = vx;
  message.vy = vy;
  message.vz = vz;

  //日志打印
  RCLCPP_INFO(this->get_logger(), "Publishing" );

  command_publisher_->publish(message);
}

void Founder::init_param() {
  this->declare_parameter("ekf.xyz", 0.0);
  this->declare_parameter("ekf.yaw", 0.0);
  this->declare_parameter("ekf.r", 0.0);
  this->declare_parameter("vx", 0);
  this->declare_parameter("vy", 0);
  this->declare_parameter("vz", 0);

  this->get_parameter("ekf.xyz", ekf_xyz);
  this->get_parameter("ekf.yaw", ekf_yaw);
  this->get_parameter("ekf.r", ekf_r);
  this->get_parameter("vx", vx);
  this->get_parameter("vy", vy);
  this->get_parameter("vz", vz);

  RCLCPP_INFO(this->get_logger(),
              "xyz=%.3f, yaw=%.3f, r=%.3f, vx=%d, vy=%d, vz=%d", ekf_xyz,
              ekf_yaw, ekf_r, vx, vy, vz);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<Founder>("founder_node");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}