#include "geometry_msgs/msg/polygon_stamped.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "pnp_solve.hpp"
#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>

#define PI 3.14159265358979323846

using namespace std;
using namespace cv;
using namespace tf2;

tf2::Vector3 tVec(0, -0.1, 0.1);    // 坐标变换位移向量
float rVec[3] = {PI / 9, 0.0, 0.0}; // 坐标变换旋转向量

class Calculate_Node : public rclcpp::Node {
public:
  // 构造函数,有一个参数为节点名称
  Calculate_Node(std::string name) : Node(name) {
    RCLCPP_INFO(this->get_logger(), "节点已启动：%s.", name.c_str());
    subscribe_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "img_points", 10,
        std::bind(&Calculate_Node::callback, this, std::placeholders::_1));
    World_Coor = {
        {-0.08, 0.04, 0}, {0.08, 0.04, 0}, {0.08, -0.04, 0}, {-0.08, -0.04, 0}};

    qua.setRPY(rVec[0], rVec[1], rVec[2]);
  }

private:
  Pnp_Solve pnp_solver;

  vector<Point3d> World_Coor; // 世界坐标系

  Quaternion qua; // 旋转四元数

  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr
      subscribe_;

  void callback(const geometry_msgs::msg::PolygonStamped::SharedPtr msg) {
    vector<Point2d> img_points;
    for (int i = 0; i < 4; i++) {
      img_points.push_back(
          Point2d(msg->polygon.points[i].x, msg->polygon.points[i].y));
    }
    pnp_solver.calculate_rtVec(World_Coor, img_points);
    Point3d locPoint = pnp_solver.output_location();
    Vector3 loc(locPoint.x, locPoint.y, locPoint.z);
    // RCLCPP_INFO(this->get_logger(), "%f", pnp_solver.calculate_distance());
    RCLCPP_INFO(this->get_logger(), "Camera_Vec: %f, %f, %f", loc.x(), loc.y(), loc.z());
    loc = TF_odom(loc);
    RCLCPP_INFO(this->get_logger(), "Odom_Vec: %f, %f, %f", loc.x(), loc.y(), loc.z());
  }

  Vector3 TF_odom(Vector3 loc) {
    Transform tfodomTrans(qua, tVec);
    Vector3 odomVector = tfodomTrans * loc;
    return odomVector;
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  /*创建对应节点的共享指针对象*/
  auto node = std::make_shared<Calculate_Node>("Calculate_01");
  /* 运行节点，并检测退出信号*/
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}