#include "pnp_solve.hpp"

void Pnp_Solve::calculate_rtVec(vector<Point3d> World_Coor,
                                vector<Point2d> Img_Coor) {
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

Point3d Pnp_Solve::output_location() {
  Point3d location = Point3d(tVec.at<double>(0, 0), tVec.at<double>(1, 0),
                             tVec.at<double>(2, 0));
  return location;
}