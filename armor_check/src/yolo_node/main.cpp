#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/dnn.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <iomanip>
#include <iostream>
#include <opencv2/dnn.hpp>
#include <sstream>

using namespace std;
using namespace cv;
using namespace cv::dnn;

// 定义模型输入图像的大小
const cv::Size kModelInputSize = {256, 256};
// 定义目标类别标签
vector<string> kClassLabels = {"B1", "B2", "B3", "B4", "B5", "B7",
                               "R1", "R2", "R3", "R4", "R5", "R7"};

class YoloNode : public rclcpp::Node {
public:
  YoloNode() : Node("yolo_node") {
    // 创建一个用于发布类别标签的 ROS 2 发布者
    label_publisher_ =
        this->create_publisher<std_msgs::msg::String>("topic1", 10);

    // 定义视频文件路径和模型路径
    string video_path =
        "/home/zkw/zkw-common/armor_tra_rec/resource/1234567.mp4";
    string model_path = "/home/zkw/ros2_ws/src/armor_check/best.onnx";

    // 加载 ONNX 模型
    detection_model_ = readNetFromONNX(model_path);

    // 打开视频文件
    video_capture_ = VideoCapture(video_path);
    if (!video_capture_.isOpened()) {
      cerr << "Error: Unable to open video file!" << endl;
      rclcpp::shutdown();
      return;
    }

    // 创建一个定时器，每20msc处理一帧视频
    timer_ = this->create_wall_timer(50ms,
                                     std::bind(&YoloNode::process_video, this));
  }

private:
  void process_video() {
    Mat current_frame;
    if (!video_capture_.read(current_frame)) {
      cerr << "Error: Frame is empty!" << endl;
      rclcpp::shutdown();
      return;
    }

    // 计算图像缩放比例
    float scale_x =
        static_cast<float>(current_frame.cols) / kModelInputSize.width;
    float scale_y =
        static_cast<float>(current_frame.rows) / kModelInputSize.height;

    // 图像预处理：将图像转换为模型输入所需的格式
    Mat input_blob =
        dnn::blobFromImage(current_frame, 1.0 / 255, kModelInputSize,
                           Scalar(0, 0, 0), true, false);

    if (input_blob.empty()) {
      cout << "Error: Input blob is empty!" << endl;
      return;
    }

    // 将预处理后的数据输入到模型中
    detection_model_.setInput(input_blob);
    // 获取模型的输出
    Mat detection_output = detection_model_.forward();

    // 将模型输出的三维矩阵转换为二维矩阵
    Mat detection_matrix =
        detection_output.reshape(1, detection_output.size[1]);

    // 定义用于存储检测结果的变量
    vector<Rect> detected_boxes;        // 候选检测框
    vector<float> detected_confidences; // 候选置信度
    vector<int> detected_class_indices; // 候选类别索引

    // 遍历检测结果
    for (int col_index = 0; col_index < detection_matrix.cols; ++col_index) {
      float max_confidence = 0.0f;
      int best_class_index = -1;

      // 遍历每个类别的置信度
      for (int row_index = 4; row_index < detection_matrix.rows; ++row_index) {
        float current_confidence =
            detection_matrix.at<float>(row_index, col_index);
        if (current_confidence > max_confidence) {
          max_confidence = current_confidence;
          best_class_index = row_index - 4;
        }
      }

      // 如果置信度大于阈值，则认为是有效的检测结果
      if (max_confidence > 0.3) {
        // 获取检测框的宽度和高度
        float box_width = detection_matrix.at<float>(2, col_index);
        float box_height = detection_matrix.at<float>(3, col_index);

        // 计算检测框的左上角坐标
        int box_x = static_cast<int>(
            (detection_matrix.at<float>(0, col_index) - box_width * 0.5) *
            scale_x);
        int box_y = static_cast<int>(
            (detection_matrix.at<float>(1, col_index) - box_height * 0.5) *
            scale_y);

        // 计算检测框的实际宽度和高度
        int box_width_scaled = static_cast<int>(box_width * scale_x);
        int box_height_scaled = static_cast<int>(box_height * scale_y);

        // 将检测框、置信度和类别索引添加到对应的列表中
        detected_boxes.push_back(
            Rect(box_x, box_y, box_width_scaled, box_height_scaled));
        detected_confidences.push_back(max_confidence);
        detected_class_indices.push_back(best_class_index);
      }
    }

    // 使用非极大值抑制（NMS）去除冗余的检测框
    vector<int> nms_indices;
    dnn::NMSBoxes(detected_boxes, detected_confidences, 0.5, 0.4, nms_indices);

    // 遍历最终的检测结果
    for (int idx : nms_indices) {
      int class_index = detected_class_indices[idx];
      stringstream label_text;
      float confidence = detected_confidences[idx];

      // 生成显示文本，包括类别标签和置信度
      label_text << kClassLabels[class_index] << " (" << fixed
                 << setprecision(2) << confidence * 100 << "%)";

      // 在图像上绘制检测框
      rectangle(current_frame, detected_boxes[idx], Scalar(0, 0, 255), 1);
      // 在图像上显示类别标签和置信度
      putText(current_frame, label_text.str(), detected_boxes[idx].tl(),
              FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 0, 255), 1);

      // 发布检测到的类别标签
      std_msgs::msg::String label_msg;
      label_msg.data = kClassLabels[class_index];
      label_publisher_->publish(label_msg);
      cout << "检测到类别标签为：" << label_msg.data << endl;
    }

    if (nms_indices.size() == 0) {
      std_msgs::msg::String label_msg;
      label_msg.data = "no armor";
      label_publisher_->publish(label_msg);
    }

    // 显示处理后的视频帧
    imshow("Video Frame", current_frame);
    waitKey(1);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr label_publisher_;
  cv::VideoCapture video_capture_;
  cv::dnn::Net detection_model_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  // 初始化 ROS 2 节点
  rclcpp::init(argc, argv);

  // 创建 YoloNode 对象
  auto yolo_node = std::make_shared<YoloNode>();

  // 运行 ROS 2 节点
  rclcpp::spin(yolo_node);

  // 关闭 ROS 2 节点
  rclcpp::shutdown();
  return 0;
}