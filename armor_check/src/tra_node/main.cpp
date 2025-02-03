#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std;
using namespace cv;

// 定义目标颜色结构体
struct TargetColor {
  string name;  // 颜色名称
  Scalar lower; // HSV 下限
  Scalar upper; // HSV 上限
};

// 定义目标颜色
TargetColor kRedColor{"red", (Scalar){0, 0, 234}, (Scalar){88, 197, 255}};
TargetColor kBlueColor{"blue", (Scalar){84, 0, 185}, (Scalar){179, 255, 255}};
TargetColor kGlareColor{"glare", (Scalar){0, 0, 200}, (Scalar){179, 80, 255}};
TargetColor kNum1Color{"num1", (Scalar){59, 28, 70}, (Scalar){118, 135, 133}};

// 函数声明
Mat GammaTransform(const Mat &srcImage, float gamma); // Gamma 变换
int CalculateDistance(Point p1, Point p2);            // 计算两点距离
vector<vector<Point>> DetectColorContours(Mat targetImg,
                                          TargetColor targetColor); // 颜色检测
Rect FindArmor(Mat &targetImg, TargetColor color,
               TargetColor numColor); // 寻找装甲板
void FindColorContours(Mat inputImg,
                       vector<vector<Point>> &contours); // 寻找颜色轮廓
void ShowColorRect(Mat &inputImg, Rect rect, TargetColor color,
                   Scalar showColor); // 显示颜色矩形框

// Gamma 变换函数
Mat GammaTransform(const Mat &srcImage, float gamma) {
  Mat lookUpTable(1, 256, CV_8U); // 创建查找表
  uchar *p = lookUpTable.ptr();
  for (int i = 0; i < 256; ++i) {
    p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
  }
  Mat resultImage;
  LUT(srcImage, lookUpTable, resultImage); // 应用查找表
  return resultImage;
}

// 计算两点之间的欧几里得距离
int CalculateDistance(Point p1, Point p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

// 检测特定颜色的轮廓
vector<vector<Point>> DetectColorContours(Mat targetImg,
                                          TargetColor targetColor) {
  Mat imgHSV, imgTargetColor;
  vector<vector<Point>> contours, filteredContours;

  // 将图像从 BGR 转换为 HSV
  cvtColor(targetImg, imgHSV, COLOR_BGR2HSV);

  // 根据颜色范围创建掩码
  inRange(imgHSV, targetColor.lower, targetColor.upper, imgTargetColor);

  // 寻找轮廓
  FindColorContours(imgTargetColor, contours);

  // 将轮廓添加到返回列表中
  for (auto contour : contours) {
    filteredContours.push_back(contour);
  }
  return filteredContours;
}

// 寻找装甲板
Rect FindArmor(Mat &targetImg, TargetColor color, TargetColor numColor) {
  Rect noArmor(0, 0, 0, 0); // 无装甲板的默认矩形
  vector<vector<Point>> colorContours, numberContours, combinedContours;
  colorContours = DetectColorContours(targetImg, color); // 寻找颜色轮廓
  numberContours = DetectColorContours(targetImg, numColor); // 寻找数字轮廓

  if (numberContours.size() <= 0)
    return noArmor; // 如果没有找到数字轮廓，返回默认矩形

  Rect light1, light2, numberRect; // 装甲板灯条
  int minDistance = 10000000;      // 最小距离
  int minDifference = 100;         // 最小距离差

  // 对于每个识别出的数字块，枚举所有灯条以判断是否为装甲板
  for (auto numberContour : numberContours) {
    Rect numberBoundingRect = boundingRect(numberContour);
    Point centerNumber = {
        (numberBoundingRect.tl().x + numberBoundingRect.br().x) / 2,
        (numberBoundingRect.tl().y + numberBoundingRect.br().y) / 2};
    if (numberBoundingRect.area() > 1000) {
      for (auto contour1 : colorContours) {
        for (auto contour2 : colorContours) {
          Rect rect1 = boundingRect(contour1);
          Rect rect2 = boundingRect(contour2);
          Point p1 = {(rect1.tl().x + rect1.br().x) / 2,
                      (rect1.tl().y + rect1.br().y) / 2};
          Point p2 = {(rect2.tl().x + rect2.br().x) / 2,
                      (rect2.tl().y + rect2.br().y) / 2};
          if (p1.x < centerNumber.x && p2.x > centerNumber.x ||
              p1.x > centerNumber.x && p2.x < centerNumber.x) {
            if (abs(abs(p1.x - centerNumber.x) - abs(p2.x - centerNumber.x)) <
                minDifference) {
              if (CalculateDistance(p1, centerNumber) < minDistance) {
                light1 = rect1;
                light2 = rect2;
                numberRect = numberBoundingRect;
                minDistance = CalculateDistance(p1, centerNumber);
                minDifference = abs(abs(p1.x - centerNumber.x) -
                                    abs(p2.x - centerNumber.x));
              }
            }
          }
        }
      }
    }
  }

  // 计算装甲板的外接矩形
  int x1 = min(light1.tl().x, light2.tl().x);
  int y1 = min(light1.tl().y, light2.tl().y);
  int x2 = max(light1.br().x, light2.br().x);
  int y2 = max(light1.br().y, light2.br().y);
  Rect armorRect(x1, y1, x2 - x1, y2 - y1);

  // 显示装甲板矩形
  ShowColorRect(targetImg, numberRect, numColor, (Scalar){0, 0, 255});
  ShowColorRect(targetImg, light1, color, (Scalar){0, 0, 255});
  ShowColorRect(targetImg, light2, color, (Scalar){0, 0, 255});
  ShowColorRect(targetImg, armorRect, numColor, (Scalar){0, 0, 255});

  return armorRect;
}

// 寻找颜色轮廓
void FindColorContours(Mat inputImg, vector<vector<Point>> &contours) {
  vector<vector<Point>> shapeContours;
  findContours(inputImg, shapeContours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  for (auto contour : shapeContours) {
    float area = contourArea(contour);
    if (area > 20) {
      vector<Point> newContour;
      float perimeter = arcLength(contour, true);
      approxPolyDP(contour, newContour, 0.02 * perimeter, true);
      contours.push_back(newContour);
    }
  }
}

// 显示颜色矩形框
void ShowColorRect(Mat &inputImg, Rect rect, TargetColor color,
                   Scalar showColor) {
  rectangle(inputImg, rect, showColor);
  putText(inputImg, color.name,
          Point(rect.tl().x, rect.tl().y + rect.height / 2), FONT_ITALIC, 0.5,
          (Scalar){0, 0, 255}, 1);
}

// 定义一个ROS 2节点类
class ArmorDetectionNode : public rclcpp::Node {
public:
  ArmorDetectionNode() : Node("armor_detection_node") {
    RCLCPP_INFO(this->get_logger(), "Armor Detection Node started.");

    // 打开视频文件
    video_path_ = "/home/zkw/zkw-common/armor_tra_rec/resource/1234567.mp4";
    video_capture_.open(video_path_);
    if (!video_capture_.isOpened()) {
      RCLCPP_ERROR(this->get_logger(), "Unable to open video file: %s",
                   video_path_.c_str());
      rclcpp::shutdown();
      return;
    }

    // 创建一个用于发布类别标签的 ROS 2 发布者
    label_publisher_ =
        this->create_publisher<std_msgs::msg::String>("topic2", 10);

    // 创建一个定时器，每50ms处理一次视频帧
    timer_ = this->create_wall_timer(
        50ms, std::bind(&ArmorDetectionNode::process_video, this));
  }

private:
  void process_video() {
    Mat current_frame;
    if (!video_capture_.read(current_frame)) {
      RCLCPP_ERROR(this->get_logger(), "Frame is empty!");
      rclcpp::shutdown();
      return;
    }

    // Gamma 变换（可选）
    // float gamma = 2;
    // current_frame = GammaTransform(current_frame, gamma);

    // 寻找装甲板
    Rect detected_bluearmor = FindArmor(current_frame, kBlueColor, kNum1Color);
    if (detected_bluearmor.area() <= 0) {
      Rect detected_redarmor = FindArmor(current_frame, kRedColor, kNum1Color);
      if (detected_redarmor.area() <= 0) {
        RCLCPP_ERROR(this->get_logger(), "No armor detected!");
        // 发布检测到的类别标签
        std_msgs::msg::String label_msg;
        label_msg.data = "no_armor";
        label_publisher_->publish(label_msg);
      } else {
        // 发布检测到的类别标签
        std_msgs::msg::String label_msg;
        label_msg.data = "Red";
        label_publisher_->publish(label_msg);
      }
    } else {
      // 发布检测到的类别标签
      std_msgs::msg::String label_msg;
      label_msg.data = "Blue";
      label_publisher_->publish(label_msg);
    }

    // 显示处理后的视频帧
    imshow("Video Frame", current_frame);
    waitKey(1);
  }

  VideoCapture video_capture_;
  string video_path_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr label_publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArmorDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}