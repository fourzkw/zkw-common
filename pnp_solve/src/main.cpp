#include "pnp_solve.hpp"
#include <iostream>

int main() {
  Pnp_Solve pnp_solver;
  Point3d w1 = {0, 0, 0}, w2 = {0.1, 0, 0}, w3 = {0.1, 0.1, 0},
          w4 = {0, 0.1, 0};
  Point2d i1 = {100, 150}, i2 = {300, 150}, i3 = {300, 300}, i4 = {100, 300};
  pnp_solver.World_Coor.push_back(w1);
  pnp_solver.World_Coor.push_back(w2);
  pnp_solver.World_Coor.push_back(w3);
  pnp_solver.World_Coor.push_back(w4);
  pnp_solver.Img_Coor.push_back(i1);
  pnp_solver.Img_Coor.push_back(i2);
  pnp_solver.Img_Coor.push_back(i3);
  pnp_solver.Img_Coor.push_back(i4);
  pnp_solver.calculate_rtVec();
  cout<<pnp_solver.calculate_distance()<<endl;
  return 0;
}