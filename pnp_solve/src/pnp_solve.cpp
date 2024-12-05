#include "pnp_solve.hpp"

Pnp_Solve::Pnp_Solve() {
  rVec = Mat::zeros(3, 1, CV_64FC1);
  tVec = Mat::zeros(3, 1, CV_64FC1);
  cam = (Mat_<double>(3, 3) << 1462.3697, 0, 398.59394, 0, 1469.68385,
         110.68997, 0, 0, 1);
  dis =
      (Mat_<double>(1, 5) << 0.003518, -0.311778, -0.016581, 0.023682, 0.0000);
}

void Pnp_Solve::calculate_rtVec() {
  // pnp解算
  solvePnP(World_Coor, Img_Coor, cam, dis, rVec, tVec, false,
           SOLVEPNP_ITERATIVE);
  //转化为矩阵
  Rodrigues(rVec, rotationMatrix);
}

double Pnp_Solve::calculate_distance() {
  double distance =
      sqrt(pow(tVec.at<double>(0, 0), 2) + pow(tVec.at<double>(1, 0), 2) +
           pow(tVec.at<double>(2, 0), 2));
  return distance;
}