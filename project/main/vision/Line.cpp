#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Vision.h"
#include "zf_common_typedef.h"

/**
 * @brief 给图像绘制黑色边框
 * @param image 输入的图像
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
void draw_border(cv::Mat& image) {
    if (image.empty()) return;
    // 画矩形框
    cv::rectangle(image, cv::Point(0, 0), cv::Point(image.cols-1, image.rows-1), cv::Scalar(0), 1);
}


/**
 * @brief 迷宫法自适应域值扫线
 * @param img 输入的灰度图像
 * @param start 起始点坐标
 * @param block_size 自适应阈值的局部区域大小，默认为 7
 * @param clip_value 阈值偏移量，类似于曝光
 * @param max_points 最大提取点数
 * @return line_result 扫线结果
 * @author Cao Xin (adapted)
 * @date 2025-05-02
 */
line_result find_lines (cv::Mat& img, cv::Point start, int block_size, int clip_value, int max_points) {
    // 入参验证
    if (img.empty() || img.channels() != 1) {
        throw std::runtime_error("Input image must be a non-empty grayscale image.");
    }
    if (block_size < 3 || block_size % 2 == 0) {
        throw std::runtime_error("block_size must be an odd number >= 3.");
    }
    if (max_points <= 0) {
        throw std::runtime_error("max_points must be positive.");
    }
    if (start.x < 0 || start.x >= img.cols || start.y < 0 || start.y >= img.rows) {
        throw std::runtime_error("Starting point is out of image bounds.");
    }

    draw_border(img);

    // 方向定义：0:上, 1:右, 2:下, 3:左
    const int dir_front[4][2] = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
    const int dir_frontleft[4][2] = {{-1, -1}, {1, -1}, {1, 1}, {-1, 1}};
    const int dir_frontright[4][2] = {{1, -1}, {1, 1}, {-1, 1}, {-1, -1}};

    line_result result;
    result.left.clear();
    result.right.clear();
    result.center.clear();

    // 计算局部区域半径
    int half = block_size / 2;

    // 左右边线巡线
    // mode-true 左手迷宫法
    // mode-false 右手迷宫法
    for (bool mode : {true, false}) {
        std::vector<cv::Point>& current_line = mode ? result.left : result.right;

        int x = start.x;
        int y = start.y;
        int dir = 0;
        int turn = 0;
        int step = 0;

        while (step < max_points && half < x && x < img.cols - half - 1 && half < y && y < img.rows - half - 1 && turn < 4) {
            // 计算局部阈值
            int local_thres = 0;
            for (int dy = -half; dy <= half; dy++) {
                for (int dx = -half; dx <= half; dx++) {
                    local_thres += img.at<uchar>(y + dy, x + dx);
                }
            }
            // 取局部的平均值作为二值化的阈值
            local_thres /= (block_size * block_size);
            local_thres -= clip_value;

            // 获取前方和侧方像素值
            int front_value = img.at<uchar>(y + dir_front[dir][1], x + dir_front[dir][0]);
            int side_value = mode
                             ? img.at<uchar>(y + dir_frontleft[dir][1], x + dir_frontleft[dir][0])
                             : img.at<uchar>(y + dir_frontright[dir][1], x + dir_frontright[dir][0]);

            if (front_value < local_thres) {
                // 前方是路径，转向
                dir = mode ? (dir + 1) % 4 : (dir + 3) % 4;
                turn++;
            } else if (side_value < local_thres) {
                // 侧方是路径，前进
                x += dir_front[dir][0];
                y += dir_front[dir][1];
                current_line.push_back({x, y});
                step++;
                turn = 0;
            } else {
                // 侧方是背景，朝侧方移动并转向
                x += mode ? dir_frontleft[dir][0] : dir_frontright[dir][0];
                y += mode ? dir_frontleft[dir][1] : dir_frontright[dir][1];
                dir = mode ? (dir + 3) % 4 : (dir + 1) % 4;
                current_line.push_back({x, y});
                step++;
                turn = 0;
            }
        }
    }

    return result;
}