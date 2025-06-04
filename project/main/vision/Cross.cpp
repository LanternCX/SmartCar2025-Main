#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <vector>

#include "Cross.h"
#include "Line.h"
#include "Vision.h"

/**
 * @brief 计算十字目标点
 */
void calc_cross_target(track_result &track, ElementType type) {
    cv::Size size = track.left.frame_size;
    int height = std::min(size.height, size.width);
    
    std::vector<int> left_x(height + 1, -1), right_x(height + 1, -1);
    for (cv::Point pt : track.left.line) {
        if (left_x[pt.y] != -1) {
            continue;
        }
        left_x[pt.y] = pt.x;
    }
    for (cv::Point pt : track.right.line) {
        if (right_x[pt.y] != -1) {
            continue;
        }
        right_x[pt.y] = pt.x;
    }

    // 找到左右两个向内拐点的最大 y 值作为锚点值
    int dist = 15;
    int threshold = 30;
    int target_y = 0;
    for (int i = 0; i < height - dist; i++) {
        int next = i + dist;
        if (right_x[i] == -1 || right_x[next] == -1) {
            continue;
        }

        int diff = right_x[next] - right_x[i];
        if (std::abs(diff) > threshold) {
            target_y = std::max(target_y, next);
        }
    }

    for (int i = 0; i < height - dist; i++) {
        int next = i + dist;
        if (left_x[i] == -1 || left_x[next] == -1) {
            continue;
        }

        int diff = left_x[next] - left_x[i];
        if (std::abs(diff) > threshold) {
            target_y = std::max(target_y, next);
        }
    }

    track.target.y = std::max(target_y - 10, 0);
}