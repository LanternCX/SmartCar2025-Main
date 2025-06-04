/**
 * @file Bin.h
 * @brief 大津法二值化相关头文件
 * @author Cao Xin
 * @date 2025-04-03
 */

#pragma once
#include <opencv2/opencv.hpp>

void otsu_binarize(const cv::Mat& src, cv::Mat& dst, cv::Point topLeft = cv::Point(-1, -1), cv::Point bottomRight = cv::Point(-1, -1)) ;
uint8_t otsu_threshold(const cv::Mat& src, cv::Point topLeft = cv::Point(-1, -1), cv::Point bottomRight = cv::Point(-1, -1)) ;