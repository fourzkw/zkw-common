#include <colorDetector.hpp>

int distance(Point p1, Point p2) {
  return (p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y);
}

ColorDetector::ColorDetector() {}

ColorDetector::~ColorDetector() {}

//寻找单种颜色
vector<vector<Point>> ColorDetector::Detectcolor(Mat targetImg,
                                                 TargetColor targetColor) {
  Mat imgHSV, imgTarColor;
  vector<vector<Point>> contours, ret_contours;

  cvtColor(targetImg, imgHSV, COLOR_BGR2HSV);

  inRange(imgHSV, targetColor.lower, targetColor.upper, imgTarColor);
  // imshow("TarColorImage", imgTarColor);

  Find_TargetColorContours(imgTarColor, contours);
  // imshow("TarColorImage", imgTarColor);

  for (auto contour : contours) {
    // Rect colorRect = boundingRect(contour);
    //  polylines(targetImg,contour,true,(Scalar){0, 0, 255});
    // Show_TargetColorRect(targetImg, colorRect, targetColor);
    ret_contours.push_back(contour);
  }
  return ret_contours;
}

//寻找装甲板
Rect ColorDetector::Find_Armor(Mat &targetImg, TargetColor color1,
                               TargetColor color2, TargetColor numcolor) {
  vector<vector<Point>> contours1, contours2, contoursNum, contours_both;
  contours1 = Detectcolor(targetImg, color1);
  contours2 = Detectcolor(targetImg, color2);
  contoursNum = Detectcolor(targetImg, numcolor);

  for (auto contour1 : contours1) {
    for (auto contour2 : contours2) {
      cv::Rect color1Rect = cv::boundingRect(contour1);
      cv::Rect color2Rect = cv::boundingRect(contour2);
      if ((color1Rect & color2Rect).area() > 5) {
        contours_both.push_back(contour2);
        // Show_TargetColorRect(targetImg, color2Rect, color1);
      }
    }
  }

  int numid = 0;
  int max_pointsnum = 0;
  for (int i = 0; i < contoursNum.size(); i++) {
    auto contour = contoursNum[i];
    if (contour.size() > max_pointsnum) {
      numid = i;
      max_pointsnum = contour.size();
    }
  }
  Rect numRect = boundingRect(contoursNum[numid]);
  Point numPl = {(numRect.tl().x + numRect.br().x) / 2,
                 (numRect.tl().y + numRect.br().y) / 2};
  int minid1, minid2;
  int mindis1 = 100000, mindis2 = 100000;
  for (int i = 0; i < contours_both.size(); i++) {
    auto contour = contours_both[i];
    if ((boundingRect(contour) & numRect).area() > 0 ||
        boundingRect(contour).area() > numRect.area())
      continue;
    Rect lightRect = boundingRect(contour);
    Point lightPl = {(lightRect.tl().x + lightRect.br().x) / 2,
                     (lightRect.tl().y + lightRect.br().y) / 2};

    if (distance(numPl, lightPl) < mindis1) {
      minid2 = minid1;
      mindis2 = mindis1;
      minid1 = i;
      mindis1 = distance(numPl, lightPl);
    } else if (distance(numPl, lightPl) < mindis2) {
      minid2 = i;
      mindis2 = distance(numPl, lightPl);
    }
  }

  Rect rect1 = boundingRect(contours_both[minid1]);
  Rect rect2 = boundingRect(contours_both[minid2]);
  int x1 = min(rect1.tl().x, rect2.tl().x);
  int y1 = min(rect1.tl().y, rect2.tl().y);
  int x2 = max(rect1.br().x, rect2.br().x);
  int y2 = max(rect1.br().y, rect2.br().y);
  Rect armor_rect(x1, y1, x2 - x1, y2 - y1);

  return armor_rect;
}

void ColorDetector::Find_TargetColorContours(Mat inputImg,
                                             vector<vector<Point>> &contours) {
  vector<vector<Point>> shape_contours;
  findContours(inputImg, shape_contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
  for (auto contour : shape_contours) {
    float area = contourArea(contour);
    if (area > 20) {
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
  putText(inputImg, color.name,
          Point(rect.tl().x, rect.tl().y + rect.height / 2), FONT_ITALIC, 0.5,
          (Scalar){0, 0, 255}, 1);
  return;
}