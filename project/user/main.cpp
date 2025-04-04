#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>
#include <unistd.h>

#include "zf_common_headfile.h"

#include "Image.h"
#include "Display.h"
#include "Motor.h"
#include "PID.h"
#include "zf_device_ips200_fb.h"
#include "zf_driver_delay.h"
#include "zf_driver_gpio.h"

#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"
#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"

void cleanup()
{
    printf("clean up...\n");
    // 关闭电机
    pwm_set_duty(MOTOR1_PWM, 0);   
    pwm_set_duty(MOTOR2_PWM, 0);
}

void sigint_handler(int signum) 
{
    printf("exit..\n");
    exit(0);
}

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

    motorInit();

    // 注册清理函数
    atexit(cleanup);

    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);

    cv::Mat frame;
    int cnt = 0;
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
        if(gpio_get_level(SWITCH_0)){
            draw_rgb_img(frame);
        }
        if(gpio_get_level(KEY_0)){
            cnt++;
        }
        std::cerr << "left motor out: " << encoder_get_count(ENCODER_2) << '\n';

    }
    
    cap.release(); // 释放视频资源
    cv::destroyAllWindows(); // 关闭窗口
    return 0;
}

int test2(){
    // display init 
    ips200_init("/dev/fb0");

    motorInit();

    // 注册清理函数
    atexit(cleanup);

    // 注册SIGINT信号的处理函数
    signal(SIGINT, sigint_handler);

    int speed = 150;
    initPID();
    leftMotorRun(15, 1);
    std::cout << "PID: " << getPID().kp << ' ' << getPID().ki << ' ' << getPID().kd << '\n';
    while (true) {
        ips200_clear();
        ips200_show_string(10, 60, std::string("KEY 0: " + std::to_string(gpio_get_level(KEY_0))).c_str());

        int switchMod = gpio_get_level(SWITCH_0) == 0 ? 1 : -1;
        ips200_show_string(10, 40, std::string("Switch Mod: " + std::to_string(switchMod)).c_str());
        if(!gpio_get_level(KEY_1)){
            speed += 10 * switchMod;
        }
        setSpeed(speed);
        ips200_show_string(10, 100, std::string("P: " + std::to_string(getPID().kp)).c_str());
        ips200_show_string(10, 120, std::string("I: " + std::to_string(getPID().ki)).c_str());
        ips200_show_string(10, 140, std::string("D: " + std::to_string(getPID().kd)).c_str());
        system_delay_ms(100);
    }
    return 0;
}
int main()   
{
    std::cout << "version: 1.0.3" << std::endl;
    return test2();
}

