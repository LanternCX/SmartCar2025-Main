#include <vector>
#include <opencv2/opencv.hpp>

#include "Math.h"

 /**
 * @file Math.cpp
 * @brief 数学相关工具
 * @author Cao Xin
 * @date 2025-04-05
 */

/**
 * @brief 限幅
 * @param a 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅之后的数据
 * @author Cao Xin
 * @date 2025-04-05
 */
float minmax(float a, float min, float max){
    if(a < min){
        return min;
    }
    if(a > max){
        return max;
    }
    return a;
}

/**
 * @brief 取最小值
 * @param a
 * @param b
 * @return a 和 b 的最小值
 * @author Cao Xin
 * @date 2025-04-05
 */
float min(float a, float b){
    return a > b ? b : a;
}

/**
 * @brief 取最大值
 * @param a
 * @param b
 * @return a 和 b 的最大值
 * @author Cao Xin
 * @date 2025-04-05
 */
float max(float a, float b){
    return a < b ? b : a;
}

/**
 * @brief 最小二乘法拟合直线
 * @param points 输入的点集
 * @return 拟合后的直线参数
 * @author Cao Xin
 * @date 2025-04-13
 */
line_params fit_line(const std::vector<cv::Point2f>& points) {
    line_params params = {0.0f, 0.0f, 0.0f, false};

    if (points.empty()) {
        return params;
    }

    // 计算统计量
    float sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
    for (const auto& p : points) {
        sum_x += p.x;
        sum_y += p.y;
        sum_xy += p.x * p.y;
        sum_xx += p.x * p.x;
    }
    float n = static_cast<float>(points.size());
    float mean_x = sum_x / n;
    float mean_y = sum_y / n;

    // 计算斜率
    float denominator = sum_xx - n * mean_x * mean_x;
    // 垂直线
    if (std::abs(denominator) < 1e-6) {
        params.is_vertical = true;
        params.c = mean_x;
    } else {
        params.slope = (sum_xy - n * mean_x * mean_y) / denominator;
        params.intercept = mean_y - params.slope * mean_x;
    }

    return params;
}


