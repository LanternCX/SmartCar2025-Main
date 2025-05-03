#include <opencv2/opencv.hpp>
#include <string>

#include "Perspective.h"
#include "zf_common_headfile.h"

#include "Main.h"
#include "Vision.h"
#include "Display.h"
#include "Motor.h"
#include "Control.h"
#include "Servo.h"
#include "Time.h"

/**
 * @file Main.cpp
 * @brief 主函数文件
 * @author Cao Xin
 * @date 2025-04-03
 */

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
    
    // Init Controller
    control_init(100, 70);

    init_perspective();

    while (true) {
        // Read Frame
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Read frame failed" << std::endl;
            break;
        }
        
        vision_result res = process_img(frame);
        int center = res.center;
        Type type = res.type;

        int width = frame.cols;

        // Control
        set_statue(type);
        to_center(center, width / 2);
    }
    return 0;   
}

int test(){
    std::cout << cv::getBuildInformation() << std::endl;
    return 0;
}
int main() {
    std::cout << "version: 1.0.3" << std::endl;
    return test();
}
