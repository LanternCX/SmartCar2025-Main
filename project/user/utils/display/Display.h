#ifndef DISPLAY_H
#define DISPLAY_H

/**
 * 屏幕操作头文件
 */

// 屏幕高度
#define IMG_HEIGHT 320
// 屏幕宽度
#define IMG_WIDTH 240
// 屏幕图像缩放
#define IMG_SCALE 1
#include <opencv2/core/mat.hpp>
void draw_rgb_img(cv::Mat frame);
void draw_gray_img(cv::Mat frame);
#endif