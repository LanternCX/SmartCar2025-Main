#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#include "zf_common_headfile.h"

#include "Perspective.h"
#include "Vision.h"
#include "Main.h"
#include "Display.h"
#include "zf_driver_gpio.h"



void draw_line(std::vector<int> line, cv::Mat& image){
    for(size_t y = 0; y < line.size(); y++){
        if (image.type() == CV_8UC3) {
            cv::circle(image, cv::Point(line[y], y), 2, cv::Vec3b(255, 255, 255), -1);
        } else {
            cv::circle(image, cv::Point(line[y], y), 2, 255, -1);
        }
    }
}

int process_img(cv::Mat frame){
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    cv::Mat bin;
    otsu_binarize(gray, bin);

    line_result line = line_detection(bin, frame);

    cv::Mat black = cv::Mat::zeros(frame.size(), CV_8UC1);
    cv::Size size(black.cols, black.rows);
    line.left = get_perspective_line(line.left, size);
    line.right = get_perspective_line(line.right, size);
    line.center = get_perspective_line(line.center, size);
    
    // Debug
    if(VISION_DEBUG && (gpio_get_level(SWITCH_0) || gpio_get_level(SWITCH_1))){
        draw_line(line.left, black);
        draw_line(line.right, black);
        draw_line(line.center, black);
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

    return line.center[line.center.size() - 10];
}

