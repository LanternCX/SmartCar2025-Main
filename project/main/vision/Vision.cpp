#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "zf_common_headfile.h"

#include "Math.h"
#include "Main.h"

#include "Perspective.h"
#include "Vision.h"
#include "Display.h"

void draw_line(std::vector<int> line, cv::Mat& image){
    for(size_t y = 0; y < line.size(); y++){
        if (image.type() == CV_8UC3) {
            cv::circle(image, cv::Point(line[y], y), 2, cv::Vec3b(255, 255, 255), -1);
        } else {
            cv::circle(image, cv::Point(line[y], y), 2, 255, -1);
        }
    }
}

bool is_line(const std::vector<int>& res, float threshold = 3.0) {
    // 提取有效点
    std::vector<cv::Point2f> points;
    for (size_t y = 0; y < res.size(); ++y) {
        if (res[y] != -1) {
            points.emplace_back(static_cast<float>(res[y]), static_cast<float>(y));
        }
    }

    // 如果点数太少，无法判断，默认为曲线
    if (points.size() < 3) {
        return false;
    }

    // 拟合直线
    line_params params = fit_line(points);

    // 计算每个点到直线的距离
    float total_distance = 0;
    if (!params.is_vertical) {
        // 非垂直线：y = mx + b
        for (const auto& p : points) {
            // 点到直线距离公式：|mx - y + b| / sqrt(m^2 + 1)
            float distance = std::abs(params.slope * p.x - p.y + params.intercept) /
                           std::sqrt(params.slope * params.slope + 1);
            total_distance += distance;
        }
    } else {
        // 垂直线：x = c
        for (const auto& p : points) {
            float distance = std::abs(p.x - params.c);
            total_distance += distance;
        }
    }

    // 计算平均偏差
    float avg_distance = total_distance / points.size();

    // 根据阈值判断
    return avg_distance < threshold;
}

vision_result process_img(cv::Mat frame){
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    cv::Mat bin;
    otsu_binarize(gray, bin);

    line_result line = line_detection(bin, frame);

    cv::Mat black = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Size size(black.rows, black.cols);
    // cv::Size size(black.cols, black.rows);
    line.left = get_perspective_line(line.left, size);
    line.right = get_perspective_line(line.right, size);
    line.center = get_perspective_line(line.center, size);

    int base_line = line.center.size() - 50;
    if(VISION_DEBUG){
        draw_line(line.left, black);
        draw_line(line.right, black);
        draw_line(line.center, black);
        cv::line(black, cv::Point(0, base_line), cv::Point(black.cols-1, base_line), cv::Scalar(255), 1);
        if(gpio_get_level(SWITCH_0) && !gpio_get_level(SWITCH_1)){
            cv::resize(black, black, cv::Size(IMG_WIDTH, IMG_HEIGHT));
            draw_gray_img(black);
        }

        if(!gpio_get_level(SWITCH_0) && gpio_get_level(SWITCH_1)){    
            cv::resize(frame, frame, cv::Size(IMG_WIDTH, IMG_HEIGHT));
            draw_rgb_img(frame);
        }
    }else{
        ips200_clear();
    }

    Type type = is_line(line.center) ? LINE : CURVE;
    return {line.center[base_line], type};
}

