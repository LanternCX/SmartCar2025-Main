#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include "zf_common_headfile.h"
#include "zf_driver_gpio.h"
#include "Image.h"
#include "Display.h"

#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"
#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"

int test(){
    cv::VideoCapture cap(0);
    // // width
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 240);
    // // height
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 320); 
    // fps
    cap.set(cv::CAP_PROP_FPS, 60);

    if (!cap.isOpened()) {
        std::cerr << "open cam 0 failed" << std::endl;
        exit(0);
        return -1;
    }
    
    // display init 
    ips200_init("/dev/fb0");

    cv::Mat frame;

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法读取视频帧或视频已结束！" << std::endl;
            break;
        }
        
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        cv::Mat bin;

        binarizeWithOtsu(gray, bin);
        lineDetection(bin, frame);

        cv::resize(frame, frame, cv::Size(IMG_WIDTH, IMG_HEIGHT));
        draw_rgb_img(frame);
    }
    
    cap.release(); // 释放视频资源
    cv::destroyAllWindows(); // 关闭窗口
    return 0;
}

int main()   
{
    std::cout << "version: 1.0.2" << std::endl;
    return test();
}

