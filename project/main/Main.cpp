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

/**
 * @brief 巡线
 */
void timer_thread() {
    while (running.load()) {
        to_center(ImageStatus.Det_True, ImageStatus.MiddleLine);
        std::this_thread::sleep_for(timer_interval);
    }
}

/**
 * @brief 初始化
 */
void init() {
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
}

/**
 * @brief 初始化以及视觉算法
 */
int run() {
    cv::Mat frame;
    cv::VideoCapture cap(0);

    init();

    while ((int)gpio_get_level(KEY_0) == 1){
        cap >> frame;

        // gray frame process
        image_cv_zip(frame);
        imageprocess();

        // rgb frame process
        resize(frame, frame, cv::Size(80, 60));
        // ring_judge(frame);
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
    }
    return 0;
}

/**
 * @brief 测试
 */
int test() {
    init();
    while (true) {
        // set_left_speed(64);
        set_right_speed(64);
    }
    return 0;
}

/**
 * @brief 主函数
 */
int main() {
    std::cout << "version: idol" << std::endl;
    return run();
}