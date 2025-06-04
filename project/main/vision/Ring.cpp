#include "Ring.h"
#include "Line.h"
#include "Track.h"
#include "Debug.h"
#include "Vision.h"
#include <iterator>
#include <opencv2/core/types.hpp>
#include <vector>

/**
 * @file Ring.cpp
 * @brief 圆环边线计算相关
 * @author Cao Xin
 * @date 2025-06-02
 */

void calc_right_ring_target(track_result &track, ElementType type);
void calc_left_ring_target(track_result &track, ElementType type);

/**
 * @brief 确定最终拟合的中线以及预锚点的 Y 值
 */
void calc_ring_target(track_result &track, ElementType type) {
    if (right_ring_type.count(type)) {
        calc_right_ring_target(track, type);
    }
    if (left_ring_type.count(type)) {
        calc_left_ring_target(track, type);
    }

    cv::Size size = track.left.frame_size;
    int height = min(size.height, size.width);
    track.target.y = height - 30;
}

/**
 * @brief 右圆环处理
 */
void calc_right_ring_target(track_result &track, ElementType type) {
    if (type == R_RING_IN) {
        track.center = track.right.center;
    } else {
        track.center = track.left.center;
    }
}

/**
 * @brief 左圆环处理
 */
void calc_left_ring_target(track_result &track, ElementType type) {
    if (type == R_RING_IN) {
        track.center = track.right.center;
    } else {
        track.center = track.left.center;
    }
}

/**
 * @brief 计算是否已经进入圆环
 * @return 是否进入圆环
 */
bool calc_is_ring_in(const track_result track) {
    cv::Size size = track.left.frame_size;
    cv::Point target = get_pre_target();
    int height = size.height;
    // int width = size.width;

    std::vector<int> left(height + 1, -1), right(height + 1, -1);
    for (cv::Point pt : track.left.line) {
        if (left[pt.y] != -1) {
            continue;
        }
        left[pt.y] = pt.x;
    }
    for (cv::Point pt : track.right.line) {
        if (right[pt.y] != -1) {
            continue;
        }
        right[pt.y] = pt.x;
    }
    int min_idx = -1;
    int min_dist = -1;
    for (int i = 0; i < height; i++) {
        if (left[i] == -1 || right[i] == -1) {
            continue;
        }
        int dist = std::abs(left[i] - right[i]);
        if (min_idx == -1) {
            min_idx = i;
            min_dist = dist;
        } else if (dist < min_dist){
            min_idx = i;
            min_dist = dist;
        }
    }
    bool res = min_idx == -1 ? false : min_idx > 280; 
    debug(min_idx, target.y);
    return res;
}