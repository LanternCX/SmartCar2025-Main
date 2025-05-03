#ifndef LINE_H
#define LINR_H

/**
 * 扫线头文件
 */

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

typedef struct{
    std::vector<cv::Point> left;
    std::vector<cv::Point> right;
    std::vector<cv::Point> center;
}line_result;

line_result find_lines (cv::Mat& img, cv::Point start, int block_size = 7, int clip_value = 10, int max_points = 1000) ;
#endif