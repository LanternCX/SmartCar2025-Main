#include <array>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utils/logger.defines.hpp>
#include <opencv2/opencv.hpp>

#include <algorithm>
#include <iostream>
#include <utility>
#include <vector>

#include "Line.h"
#include "Ring.h"
#include "Type.h"
#include "Vision.h"
#include "Math.h"
#include "Debug.h"
#include "Track.h"

/**
 * @file Track.cpp
 * @brief 扫线相关操作
 * @author Cao Xin
 * @date 2025-04-03
 */

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
    cv::rectangle(image, cv::Point(0, 0), cv::Point(image.cols - 1, image.rows - 1), cv::Scalar(0), 10);
}

/**
 * @brief 迷宫法自适应域值扫线
 * @param img 输入的灰度图像
 * @param start 起始点坐标
 * @param block_size 自适应阈值的局部区域大小, 默认 7, 奇数
 * @param max_points 最大提取点数, 默认 1000
 * @return line_result 扫线结果
 * @author Cao Xin
 * @date 2025-05-02
 */
track_result find_lines(cv::Mat img, cv::Point start, int block_size, int max_points) {
    cv::Mat temp = cv::Mat::zeros(img.size(), CV_8UC1);
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

    track_result result;
    result.left.line.clear();
    int half = block_size / 2;

    // 左右边线巡线
    // 迷宫法简介: https://zhuanlan.zhihu.com/p/391392970

    // mode-true 左手迷宫法
    // mode-false 右手迷宫法
    for (bool mode : {true, false}) {
        std::vector<cv::Point>& current_line = mode ? result.left.line : result.right.line;
        current_line.clear();

        int x = start.x;
        int y = start.y;

        otsu_binarize(img, img, {0, y - 5}, {img.cols, y + 5});
        while(x < img.cols && x >= 0){
            int val = img.at<uchar>(y, x);
            if(val > 0){
                x += mode ? -1 : 1;
            }else{
                break;
            }
        }

        int dir = 0;
        int turn = 0;
        int step = 0;
        int tag = 0;
        while (step < max_points && half < x && x < img.cols - half && half < y && y < img.rows - half && turn < 4) {
            cv::Point pts0 = {std::max(x - half, 0), std::max(y - half, 0)};
            cv::Point pts1 = {std::min(x + half, img.cols), std::min(y + half, img.rows)};

            // denoise(img, cv::Point(x, y), half);
            otsu_binarize(img, img, pts0, pts1);

            // 获取前方和侧方像素值
            int front_value = img.at<uchar>(y + dir_front[dir][1], x + dir_front[dir][0]);
            int side_value = mode
                             ? img.at<uchar>(y + dir_frontleft[dir][1], x + dir_frontleft[dir][0])
                             : img.at<uchar>(y + dir_frontright[dir][1], x + dir_frontright[dir][0]);
            
            if (front_value < 255) {
                // 前方是黑色就转弯
                dir = mode ? (dir + 1) % 4 : (dir + 3) % 4;
                turn++;
            } else if (side_value < 255) {
                // 侧方是黑色就前进
                x += dir_front[dir][0];
                y += dir_front[dir][1];
                if (y >= start.y && tag > 10) {
                    break;
                } else if (y >= start.y) {
                    continue;
                } else {
                    tag++;
                }
                current_line.push_back({x, y});
                step++;
                turn = 0;
            } else {
                // 前方和侧方都是是白色，朝侧方移动并转向
                x += mode ? dir_frontleft[dir][0] : dir_frontright[dir][0];
                y += mode ? dir_frontleft[dir][1] : dir_frontright[dir][1];
                dir = mode ? (dir + 3) % 4 : (dir + 1) % 4;
                if (y >= start.y && tag > 10) {
                    break;
                } else if (y >= start.y) {
                    continue;
                } else {
                    tag++;
                }
                current_line.push_back({x, y});
                step++;
                turn = 0;
            }
        }
    }
    return result;
}

/**
 * @brief 判断赛道元素类型
 */
ElementType calc_element_type(const track_result &track) {
    // l, r
    line_result left = track.left;
    line_result right = track.right;
    
    // 镜像右边线使得逻辑统一
    right.line = mirror_line(right.line, right.frame_size);
    
    std::array<int, 2> solt_cnt;
    std::vector<int> left_x = trans_line(left.line, left.frame_size);
    std::vector<int> right_x = trans_line(right.line, right.frame_size);

    solt_cnt[0] = get_solt_count(left_x);
    solt_cnt[1] = get_solt_count(right_x);

    // array<左, 右>, pair<向外, 向内>
    std::array<std::pair<int, int>, 2> corner_cnt;
    corner_cnt[0] = get_corner_count(left_x);
    corner_cnt[1] = get_corner_count(right_x);

    line_params left_fit_res = fit_line(left.line);
    line_params right_fit_res = fit_line(right.line);

    debug(solt_cnt);
    debug(corner_cnt);

    // 十字状态机转移
    if (cross_type.count(get_track_type())) {
        return calc_cross(track, corner_cnt);
    }
    // 入十字判断
    if (corner_cnt[0].first == 1 && corner_cnt[1].first == 1) {
        return CROSS_BEGIN;
    }

    // 右圆环状态机转移
    if (right_ring_type.count(get_track_type())) {
        return calc_right_ring(track, corner_cnt);
    }
    // 已经出环就不重复入环，然后正常判断元素
    // 入环判断
    if (corner_cnt == std::array<std::pair<int, int>, 2>{{{0, 0}, {1, 0}}}) {
        return R_RING_READY;
    }

    // 左圆环状态机转移
    if (left_ring_type.count(get_track_type())) {
        return calc_left_ring(track, corner_cnt);
    }
    // 入环判断
    if (corner_cnt ==  std::array<std::pair<int, int>, 2>{{{1, 0}, {0, 0}}}) {
        return L_RING_READY;
    }
   
    if (left_fit_res.slope > 0.15) {
        return R_CURVE;
    }

    if (right_fit_res.slope < -0.15) {
        return L_CURVE;
    }
    return LINE;
}

ElementType calc_cross(const track_result &track, const std::array<std::pair<int, int>, 2> corner_cnt) {
    if (get_track_type() == CROSS_BEGIN) {
        if (corner_cnt[0].second == 1 && corner_cnt[1].second == 1) {
            return CROSS_IN;
        } else {
            return CROSS_BEGIN;
        }
    }

    if (get_track_type() == CROSS_IN) {
        if (corner_cnt[0].second != 1 || corner_cnt[1].second != 1) {
            return LINE;
        } else {
            return CROSS_IN;
        }
    }
    return LINE;
}

/**
 * @brief 右圆环状态机转移
 */
ElementType calc_right_ring(const track_result &track, const std::array<std::pair<int, int>, 2> corner_cnt) {
    // 准备进入圆环转移到开始进入圆环
    if (get_track_type() == R_RING_READY) {
        // 右边向外的拐点丢失
        std::array<std::pair<int, int>, 2> res = {{{0, 0}, {1, 0}}};
        if (corner_cnt != res) {
            return R_RING_BEGIN;
        } else {
            return R_RING_READY;
        }
    }
    // 开始进入转移到正在进入
    if (get_track_type() == R_RING_BEGIN) {
        // 只有右边线有一个向外的拐点
        std::array<std::pair<int, int>, 2> res = {{{0, 0}, {0, 1}}};
        if (corner_cnt == res) {    
            return R_RING_IN;
        } else {
            return R_RING_BEGIN;
        }        
    }
    // 正在进入转移到已经进入
    if (get_track_type() == R_RING_IN) {
        // 两边都没有拐点
        std::array<std::pair<int, int>, 2> res = {{{0, 1}, {0, 0}}};
        if (corner_cnt == res) {
            return R_RING_RUNNING;
        } else {
            return R_RING_IN;
        }        
    }
    // 已经进入转移到准备出环
    if (get_track_type() == R_RING_RUNNING) {
        // 左边有一个向外的拐点`
        if (corner_cnt[0].first == 1 && corner_cnt[1].second == 1) {
            return R_RING_OUT;
        } else {
            return R_RING_RUNNING;
        }        
    }
    // 准备出环转移到出环结束
    if (get_track_type() == R_RING_OUT) {
        // 右边有一个向外的拐点
        if (corner_cnt[0].first != 1 || corner_cnt[1].second != 1) {
            return R_RING_END;
        } else {
            return R_RING_OUT;
        }        
    }
    // 出环结束转移到直道模式
    if (get_track_type() == R_RING_END) {
        // 两边都没有拐点
        std::array<std::pair<int, int>, 2> res = {{{0, 0}, {0, 0}}};
        if (corner_cnt == res) {
            return LINE;
        } else {
            return R_RING_END;
        }
    }
    return LINE;
}

/**
 * @brief 左圆环状态机转移
 */
ElementType calc_left_ring(const track_result &track, const std::array<std::pair<int, int>, 2> corner_cnt) {
    // 准备进入圆环转移到开始进入圆环
    if (get_track_type() == L_RING_READY) {
        std::array<std::pair<int, int>, 2> res = {{{1, 0}, {0, 0}}};
        if (corner_cnt != res) {
            return L_RING_BEGIN;
        } else {
            return L_RING_READY;
        }
    }
    // 开始进入转移到正在进入
    if (get_track_type() == L_RING_BEGIN) {
        // 只有左边线有一个向外的拐点
        std::array<std::pair<int, int>, 2> res = {{{0, 1}, {0, 0}}};
        if (corner_cnt == res) {
            return L_RING_IN;
        } else {
            return L_RING_BEGIN;
        }        
    }
    // 正在进入转移到已经进入
    if (get_track_type() == L_RING_IN) {
        // 两边都没有拐点
        std::array<std::pair<int, int>, 2> res = {{{0, 0}, {0, 0}}};
        if (corner_cnt == res) {
            return L_RING_RUNNING;
        } else {
            return L_RING_IN;
        }        
    }
    // 已经进入转移到准备出环
    if (get_track_type() == L_RING_RUNNING) {
        // 右边有一个向外的拐点
        if (corner_cnt[0].second == 1 && corner_cnt[1].first == 1) {
            return L_RING_OUT;
        } else {
            return L_RING_RUNNING;
        }        
    }
    // 准备出环转移到出环结束
    if (get_track_type() == L_RING_OUT) {
        // 左边有一个向外的拐点
        if (corner_cnt[0].second != 1 && corner_cnt[1].first != 1) {
            return L_RING_END;
        } else {
            return L_RING_OUT;
        }        
    }
    // 出环结束转移到直道模式
    if (get_track_type() == L_RING_END) {
        // 两边都没有拐点
        std::array<std::pair<int, int>, 2> res = {{{0, 0}, {0, 0}}};
        if (corner_cnt == res) {
            return LINE;
        } else {
            return L_RING_END;
        }
    }
    return LINE;
}