#ifndef PNPSOLVE_HPP
#define PNPSOLVE_HPP

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
  Pnp_Solve();
  vector<Point3d> World_Coor;
  vector<Point2d> Img_Coor;
  void calculate_rtVec(); //计算平移、旋转向量
  double calculate_distance();	//计算距离
};

#endif