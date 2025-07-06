#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>
#include <atomic>

#include "zf_common_headfile.h"

#include "Display.h"
#include "Motor.h"
#include "Control.hpp"
#include "Servo.h"
#include "Time.h"
#include "Debug.h"
#include "image.hpp"
#include "image_cv.hpp"
#include "Color.hpp"
#include "Main.hpp"
#include "zf_driver_gpio.h"

/**
 * @file Main.cpp
 * @brief 主函数文件
 * @author Cao Xin
 * @date 2025-04-03
 */

std::thread control;

std::atomic<bool> running(true);

/**
 * @brief 程序退出时清理函数
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
void cleanup() { 
    printf("clean up...\n");

    // Close control thread
    running.store(false);
    if (control.joinable()) {
        control.join();
    }

    // Power off the Motor
    pwm_set_duty(MOTOR1_PWM, 0);   
    pwm_set_duty(MOTOR2_PWM, 0);
    pwm_set_duty(SERVO_MOTOR1_PWM, 0);
}

/**
 * @brief 程序退出时清理函数
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
void sigint_handler(int signum) {
    printf("exit..\n");
    exit(0);
}

void timer_thread() {
    while (running.load()) {
        to_center(ImageStatus.Det_True, ImageStatus.MiddleLine);
        std::this_thread::sleep_for(timer_interval);
    }
}

int run() {
    cv::Mat frame;
    cv::VideoCapture cap(0);

    // Init Display
    ips200_init("/dev/fb0");

    // Register clean up function 
    atexit(cleanup);

    // Register SIGINT handler 
    signal(SIGINT, sigint_handler);

    // Init servo
    servo_init();

    // Init Motor
    motor_init();
    
    // Init Controller
    control_init(65, 65);

    // Init image opencv
    image_cv_Init();
    
    // Init image process
    Data_Settings();

    while ((int)gpio_get_level(KEY_0) == 1){
        cap >> frame;

        // gray frame process
        image_cv_zip(frame);
        imageprocess();

        // rgb frame process
        resize(frame, frame, cv::Size(80, 60));
        ring_judge(frame);
    }
    
    // control thread
    control = std::thread(timer_thread);

    // main loop (image process)
    while (true) {
        cap >> frame;

        // gray frame process
        image_cv_zip(frame);
        imageprocess();

        // rgb frame process
        resize(frame, frame, cv::Size(80, 60));
        ring_judge(frame);

        if (ImageFlag.Zebra_Flag) {
            exit(0);
        }
        // cv::Mat color_image(60, 80, CV_8UC3); // 彩色图像，60行80列，3通道

        // for (int i = 0; i < 60; ++i) {
        //     for (int j = 0; j < 80; ++j) {
        //         uchar v = img3[i][j];
        //         cv::Vec3b color;

        //         switch (v) {
        //             case 0:  color = cv::Vec3b(0, 0, 0);       break; // 黑色
        //             case 1:  color = cv::Vec3b(255, 255, 255); break; // 白色
        //             case 6:  color = cv::Vec3b(0, 0, 255);     break; // 红色 (BGR)
        //             case 7:  color = cv::Vec3b(0, 255, 0);     break; // 绿色
        //             case 8:  color = cv::Vec3b(255, 0, 0);     break; // 蓝色
        //             case 9:  color = cv::Vec3b(255, 255, 0);     break; // 蓝色
        //             default: color = cv::Vec3b(128, 128, 128); break; // 其它值设为灰色
        //         }

        //         color_image.at<cv::Vec3b>(i, j) = color;
        //     }
        // }
        // cv::resize(color_image, color_image, cv::Size(), 2.0, 2.0, cv::INTER_NEAREST);
        // draw_rgb_img(color_image);
        // debug(center);   
    }
    return 0;
}

int test() {
    while (true) {
        debug((int) gpio_get_level(KEY_0));
    }
    return 0;
}
int main() {
    std::cout << "version: idol" << std::endl;
    return run();
}