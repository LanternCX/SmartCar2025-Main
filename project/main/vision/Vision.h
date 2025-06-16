/**
 * @file Vision.cpp
 * @brief 图像处理主要操作头文件
 * @author Cao Xin
 * @date 2025-04-13
 */

#pragma once

#include "Bin.h"
#include "Type.h"
#include "Denoise.h"
#include "Line.h"
#include "Perspective.h"
#include "Ring.h"
#include "Track.h"
#include <opencv2/core/types.hpp>

#define VISION_DEBUG 1

const int TRACK_WIDTH = 140 * 2;
typedef struct{
    int center;
    ElementType type;
}vision_result;

vision_result process_img(cv::Mat frame);
void draw_line(std::vector<int> line, cv::Mat& image);
void calc_target(track_result &track, ElementType type);
cv::Point get_pre_target();