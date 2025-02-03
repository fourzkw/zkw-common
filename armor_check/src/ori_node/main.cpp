#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace cv;

class Ori_Node : public rclcpp::Node {
public:
  Ori_Node(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    // 创建订阅者，订阅topic1
    subscribe1 = this->create_subscription<std_msgs::msg::String>(
        "topic1", 10,
        std::bind(&Ori_Node::callback1, this, std::placeholders::_1));
 // 创建订阅者，订阅topic2
    subscribe2 = this->create_subscription<std_msgs::msg::String>(
        "topic2", 10,
        std::bind(&Ori_Node::callback2, this, std::placeholders::_1));

    // 打开视频文件
    video_path = "/home/zkw/zkw-common/armor_tra_rec/resource/1234567.mp4";
    video_capture.open(video_path);
    if (!video_capture.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "无法打开视频文件: %s",
                   video_path.c_str());
      return;
    }

    // 创建一个定时器，每10ms处理一次视频帧
    timer_ =
        this->create_wall_timer(50ms, std::bind(&Ori_Node::processVideo, this));
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscribe1, subscribe2;
  VideoCapture video_capture;
  string video_path;
  string message1, message2;

  void callback1(const std_msgs::msg::String::SharedPtr msg) {
    message1 = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received message from topic1: %s",
                message1.c_str());
  }

  void callback2(const std_msgs::msg::String::SharedPtr msg) {
    message2 = msg->data;
    RCLCPP_INFO(this->get_logger(), "Received message from topic2: %s",
                message2.c_str());
  }

  void processVideo() {
    Mat frame;
    if (!video_capture.read(frame)) {
      RCLCPP_INFO(this->get_logger(), "视频播放结束");
      rclcpp::shutdown();
      return;
    }

    // 在视频帧上绘制消息
    putText(frame, "Topic 1: " + message1, Point(10, 30), FONT_HERSHEY_SIMPLEX,
            0.8, Scalar(0, 255, 0), 2);
    putText(frame, "Topic 2: " + message2, Point(10, 60), FONT_HERSHEY_SIMPLEX,
            0.8, Scalar(0, 255, 0), 2);

    // 显示视频帧
    imshow("Video Frame", frame);
    waitKey(1);
  }

  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ori_Node>("Ori_Node");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}