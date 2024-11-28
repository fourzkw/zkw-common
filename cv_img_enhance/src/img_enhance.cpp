#include <cmath>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

Mat logTransform(const Mat &srcImage, double c) {
  Mat resultImage = Mat::zeros(srcImage.size(), srcImage.type());
  srcImage.convertTo(resultImage, CV_32F); // 转换为浮点型以进行对数变换
  log(resultImage + 1, resultImage); // 计算log(1 + pixel_value)
  resultImage = c * resultImage;     // 缩放结果
  normalize(resultImage, resultImage, 0, 220,
            NORM_MINMAX);                    // 归一化到0-220 避免过曝
  convertScaleAbs(resultImage, resultImage); // 转换为8位无符号整数
  return resultImage;
}

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
  Mat image = imread("../sources/pic1.png");
  if (image.empty()) {
    cout << "Could not read the image" << endl;
    return -1;
  }
  imshow("Original Image", image);

  double c = 1.0; // 对数变换中的常数c
  Mat logTransformedImage = logTransform(image, c);
  imshow("Log Transformed Image", logTransformedImage);

  float gamma = 0.5; // Gamma值
  Mat gammaCorrectedImage = gammaTransform(image, gamma);
  imshow("Gamma Corrected Image", gammaCorrectedImage);
  waitKey(0);
  return 0;
}