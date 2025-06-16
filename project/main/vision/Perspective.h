/**
 * @file Perspective.h
 * @brief 图像逆透视相关操作头文件
 * @author Cao Xin
 * @date 2025-04-13
 */

#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

const int FRAME_WIDTH = 50;
const int FRAME_HEIGHT = 50;
void init_perspective();
cv::Mat get_perspective_img(cv::Mat image);
std::vector<cv::Point> get_perspective_line(const std::vector<cv::Point>& src, cv::Size size);
cv::Point get_perspective_pt(cv::Point src);
int mark_square();
// void mark_square();