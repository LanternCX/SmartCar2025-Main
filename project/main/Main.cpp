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
    control_init();

    init_perspective();

    int speed = 45;
    while (true) {
        // Read Frame
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "Read frame failed" << std::endl;
            break;
        }
        
        int a = get_time();
        // Cvt to gray
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        int b = get_time();
        

        // Binarize
        cv::Mat bin;
        otsu_binarize(gray, bin);
        
        
        // Get Center
        cv::Mat black = cv::Mat::zeros(frame.size(), CV_8UC1);
        int center = line_detection(bin, frame, black);


        std::cout << "time spend: " << b - a << '\n';

        int width = bin.cols;

        // Control
        to_center(center, width / 2, speed);

        // Debug
        if(gpio_get_level(SWITCH_0)){
            cv::resize(frame, frame, cv::Size(IMG_WIDTH, IMG_HEIGHT));
            // draw_rgb_img(frame);
            cv::resize(black, black, cv::Size(IMG_WIDTH, IMG_HEIGHT));
            draw_gray_img(black);
        }else{
            ips200_clear();
        }
    }
    return 0;   
}

int test(){
    // Init servo
    servo_init();

    // Init Display
    ips200_init("/dev/fb0");

    // Register clean up function 
    atexit(cleanup);

    // Register SIGINT handler 
    signal(SIGINT, sigint_handler);

    while(1){
        static int x = 90;
        x += (!gpio_get_level(KEY_0)) * (gpio_get_level(SWITCH_0) ? 1 : -1);
        ips200_clear();
        ips200_show_string(10, 20, std::string("Angle: " + std::to_string(x)).c_str());
        set_servo_duty(x);
        system_delay_ms(100);
    }
}
int main() {
    std::cout << "version: 1.0.3" << std::endl;
    return run();
}
