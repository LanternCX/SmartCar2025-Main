#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Line.h"
#include "Math.h"
#include "Perspective.h"
#include "Track.h"
#include "Vision.h"
#include "Cross.h"
#include "Ring.h"
#include "Debug.h"
#include "Type.h"

/**
 * @file Vision.cpp
 * @brief 图像处理主要操作
 * @author Cao Xin
 * @date 2025-04-13
 */

/**
 * @brief 绘制边线
 * @param line 边线
 * @param image 输入的图像
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
void draw_line(std::vector<int> line, cv::Mat& image) {
    for(size_t y = 0; y < line.size(); y++) {
        if (image.type() == CV_8UC3) {
            cv::circle(image, cv::Point(line[y], y), 2, cv::Vec3b(255, 255, 255), -1);
        } else {
            cv::circle(image, cv::Point(line[y], y), 2, 255, -1);
        }
    }
}

/**
 * @brief 图像处理主函数
 * @param frame 输入的图像
 * @return 中线识别的结果
 * @author Cao Xin
 * @date 2025-04-13
 */
vision_result process_img(cv::Mat frame) {
    // 彩色转灰度
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

    // 获取长宽
    int height = frame.cols;
    int width = frame.rows;
    if (width < height) {
        std::swap(height, width);
    }

    std::vector<cv::Point> temp;

    // 扫线
    cv::Point start = {width / 2, height - 10};
    track_result track = find_lines(gray, start);
    track.left.frame_size = frame.size();
    track.right.frame_size = frame.size();

    // 过滤边缘点
    remove_bound_pts(track.right.line, temp, frame.size());
    track.right.line = temp;
    remove_bound_pts(track.left.line, temp, frame.size());
    track.left.line = temp;

    // 三角滤波
    filter_points(track.left.line, temp, 15);
    track.left.line = temp;
    filter_points(track.right.line, temp, 15);
    track.right.line = temp;

    // 等距采样
    resample_points(track.left.line, temp, track.left.sample_dist);
    track.left.line = temp;
    resample_points(track.right.line, temp, track.right.sample_dist);
    track.right.line = temp;

    // 过滤边缘点
    remove_bound_pts(track.right.line, temp, frame.size());
    track.right.line = temp;
    remove_bound_pts(track.left.line, temp, frame.size());
    track.left.line = temp;

    // 透视变换
    track.left.line = get_perspective_line(track.left.line, frame.size());
    track.right.line = get_perspective_line(track.right.line, frame.size());

    // 过滤边缘点
    remove_bound_pts(track.right.line, temp, frame.size());
    track.right.line = temp;
    remove_bound_pts(track.left.line, temp, frame.size());
    track.left.line = temp;
    
    // 等距采样
    resample_points(track.left.line, temp, track.left.sample_dist);
    track.left.line = temp;
    resample_points(track.right.line, temp, track.right.sample_dist);
    track.right.line = temp;

    // 三角滤波
    filter_points(track.left.line, temp, 35);
    track.left.line = temp;
    filter_points(track.right.line, temp, 35);
    track.right.line = temp;

    // 元素识别
    ElementType type = calc_element_type(track);
    track.type = type;
    change_type_count(type);
    calc_track_type();

    // 拟合中线
    track.left.center.clear();
    track.left.center = shift_line(track.left.line, TRACK_WIDTH / 2);
    track.right.center.clear();
    track.right.center = shift_line(track.right.line, -(TRACK_WIDTH / 2));

    // 过滤边缘点
    remove_bound_pts(track.right.center, temp, frame.size());
    track.right.center = temp;
    remove_bound_pts(track.left.center, temp, frame.size());
    track.left.center = temp;

    // 三角滤波
    filter_points(track.left.center, temp, 35);
    track.left.center = temp;
    filter_points(track.right.center, temp, 35);
    track.right.center = temp;

    // 等距采样
    resample_points(track.left.center, temp, track.left.sample_dist);
    track.left.center = temp;
    resample_points(track.right.center, temp, track.right.sample_dist);
    track.right.center = temp;
    
    // 三角滤波
    filter_points(track.left.center, temp, 35);
    track.left.center = temp;
    filter_points(track.right.center, temp, 35);
    track.right.center = temp;

    calc_target(track, get_track_type());

    debug(get_track_type());

    if (VISION_DEBUG) {
        cv::Mat left = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::Mat right = cv::Mat::zeros(frame.size(), CV_8UC1);

        cv::Mat left_center = cv::Mat::zeros(frame.size(), CV_8UC1);
        cv::Mat right_center = cv::Mat::zeros(frame.size(), CV_8UC1);

        for (auto pts : track.left.line) {
            cv::circle(left, pts, 1, 255);
        }

        for (auto pts : track.right.line) {
            cv::circle(right, pts, 1, 255);
        }

        for (auto pts : track.left.center) {
            cv::circle(left_center, pts, 1, 255);
        }

        for (auto pts : track.right.center) {
            cv::circle(right_center, pts, 1, 255);
        }

        cv::imshow("left", left);
        cv::imshow("right", right);
        // cv::imshow("right-center", right_center);
        // cv::imshow("left-center", left_center);
    }

    return {track.target.x, get_track_type()};
}

static cv::Point pre_target;

/**
 * @brief 维护状态机
 * @param track 赛道数据，传址
 * @param type 赛道元素类型
 */ 
void calc_target(track_result &track, ElementType type) {
    cv::Size size = track.left.frame_size;
    if (cross_type.count(type)) {
        calc_cross_target(track, type);
    } else if (ring_type.count(type)) {
        calc_ring_target(track, type);
    } else {
        track.center = track.left.center;
        track.target.y = size.height - 15;
    }
    
    // 找到 y 值距离预锚点 y 值最近的点作为预锚点
    int min_dist = std::max(size.height, size.width);
    for (cv::Point pts : track.center) {
        int dist = std::abs(pts.y - track.target.y);
        if (dist < min_dist) {
            track.target.x = pts.x;
            pre_target = track.target;
        }
    }
}

/**
 * @brief 获取上一次的预锚点
 */
cv::Point get_pre_target() {
    return pre_target;
}