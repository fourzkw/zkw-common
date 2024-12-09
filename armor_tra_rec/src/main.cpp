#include "Find_Color.hpp"
#include "colorDetector.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "pnp_solve.hpp"

TargetColor red{"red", (Scalar){0, 0, 234}, (Scalar){88, 197, 255}};
TargetColor blue{"blue", (Scalar){0, 0, 214}, (Scalar){130, 83, 255}};
TargetColor glare{"glare", (Scalar){0, 0, 200}, (Scalar){179, 80, 255}};
TargetColor num1{"num1", (Scalar){122, 90, 223}, (Scalar){141, 137, 255}};
TargetColor num2{"num2", (Scalar){60, 0, 166}, (Scalar){119, 54, 255}};

ColorDetector colorDetector;

Mat gammaTransform(const Mat &srcImage, float gamma) {
  Mat lookUpTable(1, 256, CV_8U);
  uchar *p = lookUpTable.ptr();
  for (int i = 0; i < 256; ++i) {
    p[i] = cv::saturate_cast<uchar>(pow(i / 255.0, gamma) * 255.0);
  }
  Mat resultImage;
  LUT(srcImage, lookUpTable, resultImage);
  return resultImage;
}

int main() {
  string path1 = "../resource/p1.png";
  string path2 = "../resource/p2.png";
  Mat p1 = imread(path1);
  Mat p2 = imread(path2);

  float gamma = 2; // Gamma值
  p1 = gammaTransform(p1, gamma);
  p2 = gammaTransform(p2, gamma);
  // imshow("Gamma Corrected Image", p2);

  Find_Color find;
  //  find.choose_hsv(p2);

  //
  Rect armor1 = colorDetector.Find_Armor(p1, red, glare, num1);
  Rect armor2 = colorDetector.Find_Armor(p2, blue, glare, num2);
  colorDetector.Show_TargetColorRect(p1, armor1, red);
  colorDetector.Show_TargetColorRect(p2, armor2, blue);
  imshow("pic1", p1);
  imshow("pic2", p2);

  Pnp_Solve pnp1, pnp2;
  Point2d a1 = {armor1.x, armor1.y}, a2 = {armor1.x + armor1.width, armor1.y},
          a3 = {armor1.x, armor1.y + armor1.height},
          a4 = {armor1.x + armor1.width, armor1.y + armor1.height};
  Point2d b1 = {armor2.x, armor2.y}, b2 = {armor2.x + armor2.width, armor2.y},
          b3 = {armor2.x, armor2.y + armor2.height},
          b4 = {armor2.x + armor2.width, armor2.y + armor2.height};
  Point3d w1 = {-0.04, 0.02, 0}, w2 = {0.04, 0.02, 0}, w3 = {-0.04, -0.02, 0},
          w4 = {0.04, -0.02, 0};
  //   Point3d w1 = {0, 0.04, 0}, w2 = {0.08, 0.04, 0}, w3 = {0, 0, 0},
  //           w4 = {0.08, 0, 0};
  pnp1.Img_Coor.push_back(a1);
  pnp1.Img_Coor.push_back(a2);
  pnp1.Img_Coor.push_back(a3);
  pnp1.Img_Coor.push_back(a4);
  pnp1.World_Coor.push_back(w1);
  pnp1.World_Coor.push_back(w2);
  pnp1.World_Coor.push_back(w3);
  pnp1.World_Coor.push_back(w4);
  pnp1.calculate_rtVec();

  cout << pnp1.calculate_distance() << endl;

  pnp2.Img_Coor.push_back(b1);
  pnp2.Img_Coor.push_back(b2);
  pnp2.Img_Coor.push_back(b3);
  pnp2.Img_Coor.push_back(b4);
  pnp2.World_Coor.push_back(w1);
  pnp2.World_Coor.push_back(w2);
  pnp2.World_Coor.push_back(w3);
  pnp2.World_Coor.push_back(w4);

  pnp2.calculate_rtVec();
  cout << pnp2.calculate_distance() << endl;
  waitKey(0);
  return 0;
}