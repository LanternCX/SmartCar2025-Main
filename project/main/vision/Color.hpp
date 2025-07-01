#pragma once
#include <opencv2/opencv.hpp>
#include <vector>
std::vector<cv::Rect> colorDetect(const cv::Mat& scr, cv::Mat& dst, const cv::Scalar& low, const cv::Scalar& high);