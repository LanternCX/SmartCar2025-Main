/**
 * @file Line.h
 * @brief 边线处理相关操作头文件
 * @author Cao Xin
 * @date 2025-05-03
 */

#pragma once

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

/**
 * @brief 边线数据
 */
typedef struct line_result {
    std::vector<cv::Point> line;
    std::vector<cv::Point> center;
    std::vector<float> slope;
    cv::Size frame_size;
    int sample_dist = 1;
} line_result;

void filter_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, int kernel);
void resample_points(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, float dist);
void get_line_slope(const std::vector<cv::Point>& pts_in, std::vector<float>& angle_out, int angle_dist, int sample_dist);
void filter_line_slope(const std::vector<float>& angle_in, std::vector<float>& angle_out, int kernel);
void rebuild_line(const std::vector<float>& angle_in, std::vector<cv::Point>& pts_out, int dist, cv::Point origin);
int get_solt_count(const std::vector<int> &line);
std::pair<int, int> get_corner_count(const std::vector<int> &line);
bool is_line(const std::vector<cv::Point>& points, const float &threshold = 200.0);
std::vector<int> trans_line(const std::vector<cv::Point> &line, const cv::Size &size);
void remove_bound_pts(const std::vector<cv::Point>& pts_in, std::vector<cv::Point>& pts_out, cv::Size size);
std::vector<cv::Point> mirror_line(const std::vector<cv::Point>& line, cv::Size size);
std::vector<cv::Point> shift_line(const std::vector<cv::Point>& line, int dist);