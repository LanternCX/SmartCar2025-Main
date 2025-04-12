#ifndef LINE_H
#define LINR_H

/**
 * 扫线头文件
 */

#include <opencv2/opencv.hpp>
#include <vector>
int line_detection(cv::Mat binary, cv::Mat& src, cv::Mat& black);
#endif