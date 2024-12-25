#pragma once

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace cv;

class Pnp_Solve {
private:
  Mat rVec;           //旋转向量
  Mat rotationMatrix; //旋转矩阵
  Mat tVec;           //平移向量
  Mat cam;            //相机内参
  Mat dis;            //畸变矩阵
public:
  Pnp_Solve() {
    rVec = Mat::zeros(3, 1, CV_64FC1);
    tVec = Mat::zeros(3, 1, CV_64FC1);
    cam = (Mat_<double>(3, 3) << 1462.3697, 0, 398.59394, 0, 1469.68385,
           110.68997, 0, 0, 1);
    dis = (Mat_<double>(1, 5) << 0.003518, -0.311778, -0.016581, 0.023682,
           0.0000);
  };
  void calculate_rtVec(vector<Point3d> World_Coor,
                       vector<Point2d> Img_Coor); //计算平移、旋转向量
  double calculate_distance();                    //计算距离
  Point3d output_location();                      //输出平移向量
};