#ifndef LINE_H
#define LINR_H

/**
 * 扫线头文件
 */

#include <opencv2/opencv.hpp>
#include <vector>

typedef struct{
    std::vector<int> left;
    std::vector<int> right;
    std::vector<int> center;
}line_result;

line_result line_detection(cv::Mat binary, cv::Mat& src);
#endif