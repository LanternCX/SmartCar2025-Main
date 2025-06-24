/**
 * @file Math.h
 * @brief 数学相关工具头文件
 * @author Cao Xin
 * @date 2025-04-05
 */

#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

/**
 * 直线参数
 */
struct line_params {
    // 斜率
    float slope;
    // 截距
    float intercept; 
    // 垂直线 x = c
    float c;
    // 是否为垂直线
    bool is_vertical;
};

float clip(float a, float min, float max);
float min(float a, float b);
float max(float a, float b);
line_params fit_line(const std::vector<cv::Point>& points);