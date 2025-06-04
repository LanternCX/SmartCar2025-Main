/**
 * @file Denoise.h
 * @brief 图像降噪相关操作头文件
 * @author Cao Xin
 * @date 2025-05-03
 */

#pragma once

#include <opencv2/opencv.hpp>
void denoise(cv::Mat& image, const cv::Point center, int size);