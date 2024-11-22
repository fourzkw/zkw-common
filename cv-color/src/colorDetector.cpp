#include <colorDetector.hpp>

ColorDetector::ColorDetector() {}

ColorDetector::~ColorDetector() {}

void ColorDetector::Detectcolor(Mat &targetImg, TargetColor targetColor) {
  Mat imgHSV, imgTarColor;
  vector<vector<Point>> contours;

  cvtColor(targetImg, imgHSV, COLOR_BGR2HSV);

  inRange(imgHSV, targetColor.lower, targetColor.upper, imgTarColor);
  //imshow("TarColorImage", imgTarColor);

  Find_TargetColorContours(imgTarColor, contours);
  // imshow("TarColorImage", imgTarColor);

  for (auto contour : contours) {
    Rect colorRect = boundingRect(contour);
    Show_TargetColorRect(targetImg, colorRect, targetColor);
  }
  return;
}

void ColorDetector::Find_TargetColorContours(Mat inputImg,
                                             vector<vector<Point>> &contours) {
  vector<vector<Point>> shape_contours;
  std::vector<cv::Vec4i> tmpH;
  findContours(inputImg, shape_contours, tmpH, RETR_EXTERNAL,
               CHAIN_APPROX_SIMPLE);
  for (auto contour : shape_contours) {
    float area = contourArea(contour);
    if (area > 1000.0) {
      vector<Point> newContour;
      float p = arcLength(contour, true);
      approxPolyDP(contour, newContour, 0.02 * p, true);
      contours.push_back(newContour);
    }
  }
  return;
}

void ColorDetector::Show_TargetColorRect(Mat &inputImg, Rect rect,
                                         TargetColor color) {
  rectangle(inputImg, rect, (Scalar){0, 0, 255});
  cv::putText(inputImg, color.name,
              cv::Point(rect.tl().x, rect.tl().y + rect.height / 2),
              cv::FONT_ITALIC, 0.5, (Scalar){0, 0, 255}, 1);
  return;
}