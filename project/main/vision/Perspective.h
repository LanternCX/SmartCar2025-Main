#ifndef _PERSPECTIVE_H_
#define _PERSPECTIVE_H_

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

const int FRAME_WIDTH = 300;
const int FRAME_HEIGHT = 300;
void init_perspective();
cv::Mat get_perspective_img(cv::Mat image);
std::vector<int> get_perspective_line(const std::vector<int>& line, cv::Size size);
cv::Point2f get_perspective_pt(cv::Point2f src);
// void mark_square();
#endif