/**
 * @file Track.cpp
 * @brief 扫线相关头文件
 * @author Cao Xin
 * @date 2025-04-03
 */

#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include "Line.h"

const bool IS_RIGHT_RING = 1;

enum ElementType {
    LINE, // 0
    L_CURVE, // 1
    R_CURVE, // 2
    CROSS_BEGIN, // 3
    CROSS_IN, // 4
    L_RING_READY, // 5
    L_RING_BEGIN, // 6
    L_RING_IN, // 7
    L_RING_RUNNING, // 8
    L_RING_OUT, // 9
    L_RING_END, // 10
    R_RING_READY, // 11
    R_RING_BEGIN, // 12
    R_RING_IN, // 13
    R_RING_RUNNING, // 14
    R_RING_OUT, // 15
    R_RING_END, // 16
};

const std::vector<ElementType> int_to_element = {
    LINE,
    L_CURVE,
    R_CURVE,
    CROSS_BEGIN,
    CROSS_IN,
    L_RING_READY,
    L_RING_BEGIN,
    L_RING_IN,
    L_RING_RUNNING,
    L_RING_OUT,
    L_RING_END,
    R_RING_READY,
    R_RING_BEGIN,
    R_RING_IN,
    R_RING_RUNNING,
    R_RING_OUT,
    R_RING_END,
};


const std::set<ElementType> left_ring_type = {
    L_RING_READY,
    L_RING_BEGIN,
    L_RING_IN,
    L_RING_RUNNING,
    L_RING_OUT,
    L_RING_END,
};
const std::set<ElementType> right_ring_type = {
    R_RING_READY,
    R_RING_BEGIN,
    R_RING_IN,
    R_RING_RUNNING,
    R_RING_OUT,
    R_RING_END,
};

const std::set<ElementType> ring_type = {
    L_RING_READY,
    L_RING_BEGIN,
    L_RING_IN,
    L_RING_RUNNING,
    L_RING_OUT,
    L_RING_END,
    R_RING_READY,
    R_RING_BEGIN,
    R_RING_IN,
    R_RING_RUNNING,
    R_RING_OUT,
    R_RING_END,
};

const std::set<ElementType> cross_type = {
    CROSS_BEGIN,
    CROSS_IN
};

/**
 * @brief 左右边线和中线
 */
typedef struct {
    // 左边线
    line_result left;
    
    // 右边线
    line_result right;
    
    // 元素类型
    ElementType type;

    // 最终拟合的中线
    std::vector<cv::Point> center;

    // 预锚点
    cv::Point target;
} track_result;

track_result find_lines(cv::Mat img, cv::Point start, int block_size = 7, int max_points = 1000);
ElementType calc_element_type(const track_result &track);
ElementType calc_cross(const track_result &track, const std::array<std::pair<int, int>, 2> corner_cnt);
ElementType calc_right_ring(const track_result &track, const std::array<std::pair<int, int>, 2> corner_cnt);
ElementType calc_left_ring(const track_result &track, const std::array<std::pair<int, int>, 2> corner_cnt);