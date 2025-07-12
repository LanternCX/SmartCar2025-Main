#include <cstdlib>
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
#include "zf_device_ips200_fb.h"
#include "zf_driver_encoder.h"
#include "zf_driver_gpio.h"

/**
 * @file Main.cpp
 * @brief 主函数文件
 * @author Cao Xin
 * @date 2025-04-03
 */

std::thread control;
std::thread test_thread;

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
    if (control.joinable() && std::this_thread::get_id() != control.get_id()) {
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
        // debug(ImageFlag.image_element_rings_flag);
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

    // Init IMU
    imu_get_dev_info();

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
        // ring_judge(frame);

        // debug(ImageFlag.image_element_rings_flag);

        if (ImageFlag.Zebra_Flag) {
            exit(0);
        }
        if ((int)gpio_get_level(KEY_1) == 0) {
            exit(0);
        }
    }
    return 0;
}

/**
 * @brief 测试
 */
int motor_sync() {
    init();
    sync_motor_duty(1900);
    return 0;
}

int motor_test() {
    init();
    int duty = 1700;
    left_motor_run(duty, 1700);
    return 0;
}

void test_thread_fun() {
    while (running.load()) {
        
    }
}

int test() {
    init();
    control = std::thread([&](){
        static int det = 0;
        while(running.load()) {
            if (gpio_get_level(KEY_1) == 0) {
                det++;
            }
            if (gpio_get_level(KEY_0) == 0) {
                det--;
            }
            set_servo_duty(87 + det);
            debug(det);
            
            speed_param speed;
            speed.center = 150;
            
            if (det < 0) {
                // 舵机打角小于 0 向右转 左轮外轮 右轮内轮
                calc_speed_det(det, speed.center, speed.right, speed.left);
            } else if (det > 0) {
                // 舵机打角大于 0 向左转 右轮外轮 左轮内轮
                calc_speed_det(det, speed.center, speed.left, speed.right);
            } if (det == 0) {
                speed.left = speed.center;
                speed.right = speed.center;
            }
            
            debug(speed.center, speed.left, speed.right);
            std::this_thread::sleep_for(std::chrono::milliseconds(20));

            set_left_speed(speed.left);
            set_right_speed(speed.right);
        }
    });
    
    while(true) {
    }
    return 0;
}

/**
 * @brief 主函数
 */
int main() {
    std::cout << "version: idol" << std::endl;
    return test();
}