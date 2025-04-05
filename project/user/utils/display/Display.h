#ifndef DISPLAY_H
#define DISPLAY_H

#define IMG_HEIGHT 320
#define IMG_WIDTH 240
#define IMG_SCALE 1
#include <opencv2/core/mat.hpp>
void draw_rgb_img(cv::Mat frame);
void draw_gray_img(cv::Mat frame);
#endif