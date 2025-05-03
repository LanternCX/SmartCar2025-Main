#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Perspective.h"

/**
 * @file 
 * @brief 实现示例功能的主程序逻辑
 * 
 * 本文件用于演示如何对 C++ 文件进行文档注释。可与 Doxygen 工具配合生成文档。
 * 
 * @author 曹鑫
 * @date 2025-05-04
 */

cv::Mat perspectiveTransform;

void init_perspective(){
    // 源点，手动标定获得，arg = 30 height = 18
    // 左上, 右上, 右下, 左下
    std::vector<cv::Point2f> src = {
        cv::Point2f(220, 267),
        cv::Point2f(418, 267),
        cv::Point2f(502, 462),
        cv::Point2f(139, 462)
    };

    // 目标点
    std::vector<cv::Point2f> dst = {
        cv::Point2f(0, 0),
        cv::Point2f(FRAME_WIDTH - 1, 0),
        cv::Point2f(FRAME_WIDTH - 1, FRAME_HEIGHT - 1),
        cv::Point2f(0, FRAME_HEIGHT - 1)
    };
    perspectiveTransform = getPerspectiveTransform(src, dst);
}

cv::Mat get_perspective_img(cv::Mat image){
    cv::Mat warped;
    warpPerspective(image, warped, perspectiveTransform, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
    // cv::warpPerspective(image, warped, perspectiveTransform, image.size());
    return warped;
}

std::vector<int> get_perspective_line(const std::vector<int>& line, cv::Size size) {
    float height = size.height;
    float width = size.width;

    // 计算中心偏移量，方便对齐
    cv::Point2f center = get_perspective_pt(cv::Point2f(width / 2, height / 2));
    float dx = center.x - width / 2;
    float dy = center.y - height / 2;
    
    // 原点集 
    std::vector<cv::Point2f> src;
    for(size_t y = 0; y < line.size(); y++) {
        src.push_back(cv::Point2f(line[y], y));
    }

    // 转换后的点集
    std::vector<cv::Point2f> dst;
    cv::perspectiveTransform(src, dst, perspectiveTransform);
    
    // 初始化结果数组，大小与输入line相同
    std::vector<int> res(line.size(), -1);
    
    // 将变换后的点映射回整数数组
    for(size_t i = 0; i < dst.size(); i++) {
        int newX = static_cast<int>(std::round( dst[i].x - dx));
        int newY = static_cast<int>(std::round(dst[i].y - dy));
        
        if(newY >= 0 && newY < static_cast<int>(res.size()) && 
           newX >= 0 && newX < width) {
            res[newY] = newX;
        }
    }
    return res;
}

cv::Point2f get_perspective_pt(cv::Point2f src){
    // 转换为输入点数组
    std::vector<cv::Point2f> srcPoints = {src};
    std::vector<cv::Point2f> dstPoints;
    
    // 执行逆透视变换
    cv::perspectiveTransform(srcPoints, dstPoints, perspectiveTransform);

    return dstPoints[0];
}