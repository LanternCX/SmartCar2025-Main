#ifndef BIN_H
#define BIN_H
#include <opencv2/opencv.hpp>

/**
 * 大津法二值化头文件
 */

void otsu_binarize(const cv::Mat& src, cv::Mat& dst);
#endif