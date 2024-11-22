#ifndef COLORDETECTOR_HPP
#define COLORDETECTOR_HPP

#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

struct TargetColor {
  string name;
  Scalar lower;
  Scalar upper;
};

class ColorDetector {
private:
  /* data */
public:
  void Detectcolor(Mat &, TargetColor);
  void Find_TargetColorContours(Mat, vector<vector<Point>> &);
  void Show_TargetColorRect(Mat &, Rect, TargetColor);
  ColorDetector();
  ~ColorDetector();
};

#endif