#include <opencv2/opencv.hpp>
#include <string>

#include "zf_common_headfile.h"

#include "Main.h"
#include "Display.h"
#include "Motor.h"
#include "Control.h"
#include "Servo.h"
#include "Time.h"
#include "Debug.h"
#include "Image.h"

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

/**
 * @brief 给图像绘制黑色边框
 * @param image 输入的图像
 * @return none
 * @author Cao Xin
 * @date 2025-04-03
 */
void draw_border(cv::Mat& image) {
    if (image.empty()) return;
    // 画矩形框
    cv::rectangle(image, cv::Point(0, 0), cv::Point(image.cols - 1, image.rows - 1), cv::Scalar(0), 10);
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
    control_init(65, 65);

    while (true) {
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "无法读取视频帧或视频已结束！" << std::endl;
            break;
        }

        // 不知道为什么 resize 会导致透视变换不可用
        // cv::resize(frame, frame, cv::Size(), 0.5, 0.5);

        double alpha = 1.0;
        int beta = 50;
        frame.convertTo(frame, -1, alpha, beta);

        
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        
        draw_border(gray);

        // Resize gray to desired size
        cv::Mat resized_gray;
        cv::resize(gray, resized_gray, cv::Size(Image_W, Image_H), 0, 0, cv::INTER_LINEAR);

        // 复制到 Original_Image
        for (int i = 0; i < Image_H; i++) {
            for (int j = 0; j < Image_W; j++) {
                Original_Image[i][j] = resized_gray.at<uchar>(i, j);
            }
        }

        Image_Process();

        int err = Image_Erro;
        debug(err);

        to_center(err, 74);
    }
    return 0;   
}

int test() {
    // Init servo
    servo_init();

    // Init Motor
    motor_init();

    // Register clean up function 
    atexit(cleanup);

    // Register SIGINT handler 
    signal(SIGINT, sigint_handler);

    while(1) {
        set_left_speed(100);
        set_right_speed(100);
    }

    return 0;
}
int main() {
    std::cout << "version: 1.0.3" << std::endl;
    return run();
}