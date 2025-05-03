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

bool is_line(const std::vector<cv::Point2f>& points, float threshold = 3.0) {
    // 点数太少默认为曲线
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

    int height = frame.cols;
    int width = frame.rows;
    if(width < height){
        std::swap(height, width);
    }

    cv::Point start = {width / 2, 10};
    line_result line = find_lines(frame, start);

    if(VISION_DEBUG){
        for(cv::Point pts : line.left){
            cv::circle(frame, pts, 2, cv::Vec3b(255, 0, 0), -1);
        }

        for(cv::Point pts : line.right){
            cv::circle(frame, pts, 2, cv::Vec3b(0, 255, 0), -1);
        }
    }

    return {width / 2, LINE};
}

