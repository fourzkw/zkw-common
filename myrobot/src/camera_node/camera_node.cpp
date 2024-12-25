#include "colorDetector.hpp"
#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;

class Camera_Node : public rclcpp::Node {

private:
  TargetColor red{"red", (Scalar){0, 0, 234}, (Scalar){88, 197, 255}};
  TargetColor blue{"blue", (Scalar){0, 0, 214}, (Scalar){130, 83, 255}};
  TargetColor glare{"glare", (Scalar){0, 0, 200}, (Scalar){179, 80, 255}};
  TargetColor num1{"num1", (Scalar){122, 90, 223}, (Scalar){141, 137, 255}};
  TargetColor num2{"num2", (Scalar){60, 0, 166}, (Scalar){119, 54, 255}};
  ColorDetector colorDetector;
  // 声明话题发布者
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr publisher_;
  // 声名定时器指针
  rclcpp::TimerBase::SharedPtr timer_;

  void timer_callback() {
    // 创建消息
    string path = "/home/zkw/zkw-common/armor_tra_rec/resource/p1.png";
    Mat p1 = imread(path);

    // gamma变换
    float gamma = 2;
    p1 = gammaTransform(p1, gamma);
    Rect armor1 = colorDetector.Find_Armor(p1, red, glare, num1);

    auto rectangle_msg = geometry_msgs::msg::PolygonStamped();
    rectangle_msg.header.stamp = this->get_clock()->now();
    rectangle_msg.header.frame_id = "base_link";

    geometry_msgs::msg::Point32 point1, point2, point3, point4;
    point1.x = (float)armor1.x;
    point1.y = (float)armor1.y;
    point1.z = 0.0;
    point2.x = (float)armor1.x + (float)armor1.width;
    point2.y = (float)armor1.y;
    point2.z = 0.0;
    point3.x = (float)armor1.x + (float)armor1.width;
    point3.y = (float)armor1.y + (float)armor1.height;
    point3.z = 0.0;
    point4.x = (float)armor1.x;
    point4.y = (float)armor1.y + (float)armor1.height;
    point4.z = 0.0;

    rectangle_msg.polygon.points.push_back(point1);
    rectangle_msg.polygon.points.push_back(point2);
    rectangle_msg.polygon.points.push_back(point3);
    rectangle_msg.polygon.points.push_back(point4);

    publisher_->publish(rectangle_msg);
    RCLCPP_INFO(this->get_logger(), "Publishing rectangle: %f, %f", rectangle_msg.polygon.points[0].x, rectangle_msg.polygon.points[0].y);
  }

public:
  Camera_Node(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());

    publisher_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>(
        "img_points", 10);

    timer_ =
        this->create_wall_timer(std::chrono::milliseconds(500),
                                std::bind(&Camera_Node::timer_callback, this));

    //
  }
  Mat gammaTransform(const Mat &srcImage, float gamma);
};

Mat Camera_Node::gammaTransform(const Mat &srcImage, float gamma) {
  Mat lookUpTable(1, 256, CV_8U);
  uchar *p = lookUpTable.ptr();
  for (int i = 0; i < 256; ++i) {
    p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
  }
  Mat resultImage;
  LUT(srcImage, lookUpTable, resultImage);
  return resultImage;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Camera_Node>("camera_01");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
