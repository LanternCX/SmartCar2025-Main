#include <cstdlib>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include "Denoise.h"
#include "Math.h"

/**
 * @file Denoise.cpp
 * @brief 图像降噪相关操作
 * @author Cao Xin
 * @date 2025-05-03
 */

/**
 * @brief 2x2 最小池化
 * @param src 输入图像
 * @param dst 输出图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void minPool2x2(cv::Mat& src, cv::Mat& dst) {
    CV_Assert(src.type() == CV_8UC1);
    dst = cv::Mat::zeros(src.size(), CV_8UC1);

    for (int y = 0; y < src.rows; ++y) {
        for (int x = 0; x < src.cols; ++x) {
            int minVal = 255;
            for (int dy = 0; dy < 2; ++dy) {
                for (int dx = 0; dx < 2; ++dx) {
                    int ny = y + dy;
                    int nx = x + dx;
                    if (ny < src.rows && nx < src.cols) {
                        minVal = std::min(minVal, static_cast<int>(src.at<uchar>(ny, nx)));
                    }
                }
            }
            dst.at<uchar>(y, x) = static_cast<uchar>(minVal);
        }
    }
}

/**
 * @brief 3*3 高斯模糊
 * @param dst 输出图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void gaussianBlur3x3(cv::Mat& src, const cv::Point center, int size) {
    CV_Assert(src.type() == CV_8UC1);
    int height = src.size().height;
    int width = src.size().width;
    
    cv::Mat dst = cv::Mat::zeros(cv::Size(size, size), CV_8UC1);

    int x0 = center.x;
    int y0 = center.y;

    int x1 = clip(x0 - size, 1, width - 2);
    int y1 = clip(y0 - size, 1, height - 2);

    int x2 = clip(x0 + size, 1, width - 2);
    int y2 = clip(y0 + size, 1, height - 2);

    // 按照高斯分布加权平均
    const int kernel[3][3] = {
        {1, 2, 1},
        {2, 4, 2},
        {1, 2, 1}
    };

    for (int y = y1; y <= y2; y++) {
        for (int x = y1; x <= x2; x++) {
            int sum = 0;
            int weightSum = 0;
            for (int dy = -1; dy <= 1; ++dy) {
                for (int dx = -1; dx <= 1; ++dx) {
                    int val = src.at<uchar>(y + dy, x + dx);
                    int w = kernel[dy + 1][dx + 1];
                    sum += val * w;
                    weightSum += w;
                }
            }
            dst.at<uchar>(y % size, x % size) = static_cast<uchar>(sum / weightSum);
        }
    }

    // 写回原图像
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            src.at<uchar>(y, x) = dst.at<uchar>(y % size, x % size);
        }
    }
}

/**
 * @brief 3*3 腐蚀
 * @param src 输入图像
 * @param dst 输出图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void erode3x3(cv::Mat& src, const cv::Point center, const int size) {
    CV_Assert(src.type() == CV_8UC1);
    int height = src.size().height;
    int width = src.size().width;
    
    int len = (size * 2 + 1);

    cv::Mat dst = cv::Mat::zeros(cv::Size(len, len), CV_8UC1);

    int x0 = center.x;
    int y0 = center.y;

    int x1 = clip(x0 - size, 1, width - 2);
    int y1 = clip(y0 - size, 1, height - 2);

    int x2 = clip(x0 + size, 1, width - 2);
    int y2 = clip(y0 + size, 1, height - 2);


    // 处理内层的每个像素
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            uchar minVal = 255;

            // 在3x3窗口内寻找最大值
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    minVal = std::min(minVal, src.at<uchar>(y + dy, x + dx));
                }
            }

            dst.at<uchar>(y % len, x % len) = minVal;
        }
    }
    // 写回原图像
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            src.at<uchar>(y, x) = dst.at<uchar>(y % len, x % len);
        }
    }
}

/**
 * @brief 3*3 膨胀
 * @param src 输入图像
 * @param dst 输出图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void dilate3x3(cv::Mat& src, const cv::Point center, const int size) {
    CV_Assert(src.type() == CV_8UC1);
    int height = src.size().height;
    int width = src.size().width;
    
    int len = (size * 2 + 1);

    cv::Mat dst = cv::Mat::zeros(cv::Size(len, len), CV_8UC1);

    int x0 = center.x;
    int y0 = center.y;

    int x1 = clip(x0 - size, 1, width - 2);
    int y1 = clip(y0 - size, 1, height - 2);

    int x2 = clip(x0 + size, 1, width - 2);
    int y2 = clip(y0 + size, 1, height - 2);


    // 处理内层的每个像素
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            uchar maxVal = 0;

            // 在3x3窗口内寻找最大值
            for (int dy = -1; dy <= 1; dy++) {
                for (int dx = -1; dx <= 1; dx++) {
                    maxVal = std::max(maxVal, src.at<uchar>(y + dy, x + dx));
                }
            }

            dst.at<uchar>(y % len, x % len) = maxVal;
        }
    }
    // 写回原图像
    for (int y = y1; y <= y2; y++) {
        for (int x = x1; x <= x2; x++) {
            src.at<uchar>(y, x) = dst.at<uchar>(y % len, x % len);
        }
    }
}

/**
 * @brief 图像降噪
 * @param image 输入图像
 * @return none
 * @author Cao Xin
 * @date 2025-05-03
 */
void denoise(cv::Mat& image, const cv::Point center, const int size) {
    CV_Assert(!image.empty() && image.type() == CV_8UC1);

    // 高斯模糊
    // gaussianBlur3x3(image, center, size);

    // cv::GaussianBlur(image, image, cv::Size(3, 3), 0);

    // 腐蚀
    erode3x3(image, center, size);
    // cv::erode(image, image, cv::Mat());

    // 膨胀
    dilate3x3(image, center, size);
    // cv::dilate(image, image, cv::Mat());


    // 最小池化
    // minPool2x2(dilated, pooled);
}