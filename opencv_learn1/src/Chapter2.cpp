#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

int main() {
  string path = "../Resources/test.png";
  Mat img = imread(path);
  Mat imgGrey, imgBlur, imgCanny, imgDil, imgErode;
  cvtColor(img, imgGrey, COLOR_BGR2GRAY);
  GaussianBlur(img, imgBlur, Size(7, 7), 5, 0);
  Canny(img, imgCanny, 50, 150);
  Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
  dilate(imgCanny, imgDil, kernel);
  erode(imgDil, imgErode, kernel);

  imshow("image", img);
  imshow("image Gray", imgGrey);
  imshow("image Blur", imgBlur);
  imshow("image Canny", imgCanny);
  imshow("image Dilation", imgDil);
  imshow("image imgErode", imgErode);
  waitKey(0);
  return 0;
}