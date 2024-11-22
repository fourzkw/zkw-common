#include <colorDetector.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

ColorDetector colorDetector;
TargetColor blue{"blue", (Scalar){97, 98, 0}, (Scalar){113, 255, 188}};
TargetColor yellow{"yellow", (Scalar){21, 120, 92}, (Scalar){34, 255, 255}};

Mat img;

int main() {
  string path = "../Resources/picture3.png";
  img = imread(path);
  colorDetector.Detectcolor(img, blue);
  colorDetector.Detectcolor(img, yellow);
  imshow("image", img);
  waitKey(0);
  return 0;
}