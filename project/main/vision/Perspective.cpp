#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "Perspective.h"

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

cv::Mat get_perspective(cv::Mat image){
    cv::Mat warped;
    warpPerspective(image, warped, perspectiveTransform, cv::Size(FRAME_WIDTH, FRAME_HEIGHT));
    // cv::warpPerspective(image, warped, perspectiveTransform, image.size());
    return warped;
}

std::vector<int> get_perspective_line(cv::Mat& image, const std::vector<int>& line) {
    // 确保输入图像是黑色的
    CV_Assert(image.type() == CV_8UC3 || image.type() == CV_8UC1);
    
    float height = image.rows;
    float width = image.cols;
    
    // 计算图像中心点并进行透视变换
    cv::Point2f center = get_perspective_pt(cv::Point2f(width / 2, height / 2));
    
    // 计算水平偏移量，使变换后的中心对齐原图像中线
    float dx = center.x - width / 2;

    // 创建输出数组
    std::vector<int> transformedLine(line.size());

    // 处理每个点
    for (size_t y = 0; y < line.size(); ++y) {
        if (line[y] < 0) {
            transformedLine[y] = -1;
            continue;
        }
        // 原始点
        cv::Point2f srcPoint(line[y], static_cast<float>(y));
        
        // 进行透视变换
        cv::Point2f transformedPoint = get_perspective_pt(srcPoint);
        
        // 对齐到中线
        transformedPoint.x -= dx;

        // 存储到输出数组
        transformedLine[y] = static_cast<int>(std::round(transformedPoint.x));
        
        // 在图像上绘制点（如果在图像范围内）
        if (transformedPoint.x >= 0 && transformedPoint.x < image.cols &&
            transformedPoint.y >= 0 && transformedPoint.y < image.rows) {
            if (image.type() == CV_8UC3) {
                image.at<cv::Vec3b>(y, static_cast<int>(transformedPoint.x)) = cv::Vec3b(255, 255, 255);
            } else {
                image.at<uchar>(y, static_cast<int>(transformedPoint.x)) = 255;
            }
        }
    }
    
    return transformedLine;
}

cv::Point2f get_perspective_pt(cv::Point2f src){
    // 转换为输入点数组
    std::vector<cv::Point2f> srcPoints = {src};
    std::vector<cv::Point2f> dstPoints(srcPoints);
    
    // 执行逆透视变换
    cv::perspectiveTransform(srcPoints, dstPoints, perspectiveTransform);
    return dstPoints[0];
}