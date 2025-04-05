#ifndef BIN_H
#define BIN_H
#include <opencv2/opencv.hpp>
void binarizeWithOtsu(const cv::Mat& src, cv::Mat& dst);
#endif