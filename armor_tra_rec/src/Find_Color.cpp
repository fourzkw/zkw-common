#include "Find_Color.hpp"

void Find_Color::choose_hsv(cv::Mat img) {
  namedWindow("trackbars", (640, 400));
  createTrackbar("hmin", "trackbars", &hmin, 179);
  createTrackbar("hmax", "trackbars", &hmax, 179);
  createTrackbar("smin", "trackbars", &smin, 255);
  createTrackbar("smax", "trackbars", &smax, 255);
  createTrackbar("vmin", "trackbars", &vmin, 255);
  createTrackbar("vmax", "trackbars", &vmax, 255);
  cvtColor(img,img,cv::COLOR_BGR2HSV);
  while (1) {
	Mat outimg;
	Scalar l = {hmin,smin,vmin};
	Scalar u = {hmax,smax,vmax};
	inRange(img,l,u,outimg);
    imshow("colorimg", outimg);
    waitKey(1);
  }
}