#include"base.hpp"
#include"image.hpp"
#include"imgdeal.hpp"
#include "steer.hpp"
#include "motor.hpp"
#include "timer.hpp"
#include "register.hpp"
#include "encoder.hpp"

#include "Debug.h"

long szr;
// PWM_GTIM steer_test(89, 0b11, 3, 20000000, 0);
// // 定义定时器间隔（5ms）
// const auto timer_interval = std::chrono::milliseconds(20);
// ENCODER encoder_l(0, 51);//左编码器初始化
// ENCODER encoder_r(3, 50);//右编码器初始化
// PWM_ls test_l(1, 10000, 8000);//这里的占空比是低电位所占比例  改周期时要将上方setDutyCycle中的10000也改掉
// PWM_ls test_r(2, 10000, 8000);//这里的占空比是低电位所占比例  改周期时要将上方setDutyCycle中的10000也改掉
// GPIO motor_l(73);//左电机方向脚初始化
// GPIO motor_r(72);//右电机方向脚初始

void timer_thread()
{
    while (true)
    {
        // speed_l = encoder_l.pulse_counter_update();
        // speed_r = -encoder_r.pulse_counter_update();
        // Motor_Control();//电机PWM转换
        // speed_l = encoder_l.pulse_counter_update();
        // speed_r = -encoder_r.pulse_counter_update();
        // if (Speed_PID_OUT_r >= 0)
        // {
        //     motor_r.setValue(0); test_r.setDutyCycle(Speed_PID_OUT_r);
        // }
        // else { motor_r.setValue(1); test_r.setDutyCycle(-Speed_PID_OUT_r); }
        // if (Speed_PID_OUT_l >= 0)
        // {
        //     motor_l.setValue(0); test_l.setDutyCycle(Speed_PID_OUT_l);
        // }
        // else { motor_l.setValue(1); test_l.setDutyCycle(-Speed_PID_OUT_l); }

        // SteerPID_Realize(ImageStatus.Det_True - ImageStatus.MiddleLine);//舵机PID计算
        // steer_test.setDutyCycle(PWM_STEER);
        // std::this_thread::sleep_for(timer_interval);

        debug(ImageStatus.Det_True);
    }
}


int main()
{
    //PWM_ATIM steer_test(89, 0b11, 3, 20000000, 1700,1);
    debug(1);
    Mat img, img_erzhi;
    int fps_count = 0;
    auto start_time = high_resolution_clock::now(); // 开始时间
    VideoCapture cap(0);
    if (!cap.isOpened()) { cerr << "Could not open the camera" << endl; return -1; }
    InitMH();
    string command;
    // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    // cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);  // 设置分辨率宽度为640
    // cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240); // 设置分辨率高度为480
    // cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // 设置视频编码格式为MJPG
    // cap.set(cv::CAP_PROP_FPS, 120); // 设置帧率为120 FPS
    // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);   // 关闭自动曝光
    // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);   //再关闭自动曝光
    // cap.set(cv::CAP_PROP_EXPOSURE, 156);        // 曝光
    std::thread threadA(timer_thread);
    threadA.detach(); // 分离线程，允许后台运行
    while (true)
    {
        // 从摄像头读取一帧图像
        cap >> img; if (img.empty()) { break; }

        img_erzhi = erzhihua(img);
        fps_count++;
        if (fps_count % 5 == 0)// 每10帧计算一次FPS
        {
            auto end_time = high_resolution_clock::now(); // 结束时间
            duration<double> time_span = end_time - start_time; // 计算时间差
            fps = fps_count / time_span.count(); // 计算帧率
            fps_count = 0; // 重置帧数计数
            start_time = high_resolution_clock::now(); // 重置开始时间
        }

        imageprocess();//图像处理

        //test_l.disable();    
    }
    cap.release();
    return 0;
}