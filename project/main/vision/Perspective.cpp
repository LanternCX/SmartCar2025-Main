#include <cmath>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Vision.h"
#include "Perspective.h"

/**
 * @file Perspective.cpp
 * @brief 图像逆透视相关操作
 * @author Cao Xin
 * @date 2025-04-13
 */

static cv::Mat perspectiveTransform;

/**
 * @brief 初始化逆透视矩阵
 * @return none
 * @author Cao Xin
 * @date 2025-04-13
 */
void init_perspective() {
    // 源点，手动标定获得，arg = 30 height = 18
    // 左上, 右上, 右下, 左下
    std::vector<cv::Point2f> src = {
        cv::Point2f(271, 317),
        cv::Point2f(372, 317),
        cv::Point2f(393, 387),
        cv::Point2f(252, 387) 
    };

    // 目标点
    std::vector<cv::Point2f> dst = {
        cv::Point2f(0, 0),
        cv::Point2f(FRAME_WIDTH - 1, 0),
        cv::Point2f(FRAME_WIDTH - 1, FRAME_HEIGHT - 1),
        cv::Point2f(0, FRAME_HEIGHT - 1)
    };
    perspectiveTransform = getPerspectiveTransform(src, dst);
    if (VISION_DEBUG) {
        std::cout << "perspectiveTransform: \n";
        std::cout << perspectiveTransform << '\n';
    }
}

/**
 * @brief 对图像进行透视变换
 * @param image 输入的图像
 * @return 透视变换之后的图像
 * @author Cao Xin
 * @date 2025-04-13
 */
cv::Mat get_perspective_img(cv::Mat frame) {
    std::vector<cv::Point2f> srcPoints = {
        cv::Point2f(271, 317),
        cv::Point2f(372, 317),
        cv::Point2f(393, 387),
        cv::Point2f(252, 387) 
    };

    std::vector<cv::Point2f> dstPoints = {
        cv::Point2f(0, 0),
        cv::Point2f(FRAME_WIDTH - 1, 0),
        cv::Point2f(FRAME_WIDTH - 1, FRAME_HEIGHT - 1),
        cv::Point2f(0, FRAME_HEIGHT - 1)
    };

    cv::Mat M = getPerspectiveTransform(srcPoints, dstPoints);

    // 用计算出来的透视变换矩阵对整幅图像进行变换
    cv::Mat warped;
    // 输出图像尺寸与原图一致
    warpPerspective(frame, warped, M, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));

    return warped;
}

/**
 * @brief 对点集进行透视变换
 * @param line 输入的点集
 * @param size 原图的长宽
 * @return 透视变换之后的点集
 * @author Cao Xin
 * @date 2025-04-13
 */
std::vector<cv::Point> get_perspective_line(const std::vector<cv::Point>& src, cv::Size size) {
    float height = size.height;
    float width = size.width;

    if (height > width) {
        // std::cout << "heigth swap" << '\n';
        std::swap(height, width);
    }

    // 计算中心偏移量，方便对齐
    cv::Point center = get_perspective_pt(cv::Point(width / 2, height / 2));
    int dx = center.x - width / 2;
    int dy = center.y - height / 2;

    int margin = 10;
    // 转换后的点集
    std::vector<cv::Point> dst;
    for(cv::Point pt : src) {
        
        cv::Point res = get_perspective_pt(pt);
        // // 边缘全丢线的点没有意义直接丢弃
        // if (pt.y < margin && pt.x < margin || pt.y > height - margin && pt.x > width - margin) {
        //     continue;
        // }

        // // 在 x 方向上丢线只对 y 进行透视变换
        // if (pt.x < margin || pt.x > width - margin) {
        //     dst.push_back(cv::Point(pt.x, res.y - dy));
        //     continue;
        // }

        // // 在 y 方向上丢线只对 x 进行透视变换
        // if (pt.y < margin  || pt.y > height - margin) {
        //     dst.push_back(cv::Point(pt.x - dx, res.y));
        //     continue;
        // }
        
        dst.push_back(cv::Point(res.x - dx, res.y - dy));
    }
    return dst;
}

/**
 * @brief 对点进行透视变换
 * @param line 输入的点集
 * @param size 原图的长宽
 * @return 透视变换之后的点集
 * @author Cao Xin
 * @date 2025-04-13
 */
cv::Point get_perspective_pt(cv::Point src) {
    // 转换为输入点数组
    std::vector<cv::Point2f> srcPoints = {src};
    std::vector<cv::Point2f> dstPoints;
    
    // 执行逆透视变换
    cv::perspectiveTransform(srcPoints, dstPoints, perspectiveTransform);
    return dstPoints[0];
}