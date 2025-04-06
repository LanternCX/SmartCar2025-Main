#include <opencv2/opencv.hpp>

#include "zf_common_headfile.h"

#include "Image.h"
#include "Display.h"
#include "Motor.h"
#include "Servo.h"
#include "zf_device_ips200_fb.h"

#define KEY_0       "/dev/zf_driver_gpio_key_0"
#define KEY_1       "/dev/zf_driver_gpio_key_1"
#define KEY_2       "/dev/zf_driver_gpio_key_2"
#define KEY_3       "/dev/zf_driver_gpio_key_3"
#define SWITCH_0    "/dev/zf_driver_gpio_switch_0"
#define SWITCH_1    "/dev/zf_driver_gpio_switch_1"

/**
 * @brief 程序退出时清理函数
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
void cleanup() {
    printf("clean up...\n");

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

int run() {
    // Init cam
    cv::VideoCapture cap(0);
    if (!cap.isOpened()) {
        std::cerr << "open cam 0 failed" << std::endl;
        exit(0);
        return -1;
    }
    cv::Mat frame;

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

    int speed = 150;
    while (true) {
        // Read Frame
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Read frame failed" << std::endl;
            break;
        }

        // Cvt to gray
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        // Binarize
        cv::Mat bin;
        otsu_binarize(gray, bin);

        // Get Center
        int center = line_detection(bin, frame);

        int width = bin.cols;

        // Servo control
        servo_to_center(center, width / 2);

        // Motor control
        motor_to_center(center, width / 2, speed);

        // Debug
        if(gpio_get_level(SWITCH_0)){
            cv::resize(frame, frame, cv::Size(IMG_WIDTH, IMG_HEIGHT));
            draw_rgb_img(frame);
        }else{
            ips200_clear();

        }
    }
    return 0;   
}
int main() {
    std::cout << "version: 1.0.3" << std::endl;
    return run();
}
