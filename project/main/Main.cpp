#include"base.hpp"
#include"image.hpp"
#include"imgdeal.hpp"
#include "steer.hpp"
#include "motor.hpp"
#include "timer.hpp"
#include "register.hpp"
#include "encoder.hpp"


long szr;
PWM_GTIM steer_test(89, 0b11, 3, 20000000, 0);
// 定义定时器间隔（5ms）
const auto timer_interval = std::chrono::milliseconds(20);
ENCODER encoder_l(0, 51);//左编码器初始化
ENCODER encoder_r(3, 50);//右编码器初始化
PWM_ls test_l(1, 10000, 8000);//这里的占空比是低电位所占比例  改周期时要将上方setDutyCycle中的10000也改掉
PWM_ls test_r(2, 10000, 8000);//这里的占空比是低电位所占比例  改周期时要将上方setDutyCycle中的10000也改掉
GPIO motor_l(73);//左电机方向脚初始化
GPIO motor_r(72);//右电机方向脚初始
void timer_thread()
{
    while (true)
    {
        speed_l = encoder_l.pulse_counter_update();
        speed_r = -encoder_r.pulse_counter_update();
        Motor_Control();//电机PWM转换
        speed_l = encoder_l.pulse_counter_update();
        speed_r = -encoder_r.pulse_counter_update();
        if (Speed_PID_OUT_r >= 0)
        {
            motor_r.setValue(0); test_r.setDutyCycle(Speed_PID_OUT_r);
        }
        else { motor_r.setValue(1); test_r.setDutyCycle(-Speed_PID_OUT_r); }
        if (Speed_PID_OUT_l >= 0)
        {
            motor_l.setValue(0); test_l.setDutyCycle(Speed_PID_OUT_l);
        }
        else { motor_l.setValue(1); test_l.setDutyCycle(-Speed_PID_OUT_l); }

        SteerPID_Realize(ImageStatus.Det_True - ImageStatus.MiddleLine);//舵机PID计算
        steer_test.setDutyCycle(PWM_STEER);
        std::this_thread::sleep_for(timer_interval);
    }
}


int main()
{
    //PWM_ATIM steer_test(89, 0b11, 3, 20000000, 1700,1);
    Mat img, img_erzhi;
    int fps_count = 0;
    auto start_time = high_resolution_clock::now(); // 开始时间
    VideoCapture cap(0);
    if (!cap.isOpened()) { cerr << "Could not open the camera" << endl; return -1; }



    test_l.enable();
    test_r.enable();
    test_l.setPeriod(10000);//改周期时要将上方setDutyCycle中的10000也改掉
    test_r.setDutyCycle(2000);//这个里的占空比是指高电位所占比例
    test_l.setPeriod(10000);//改周期时要将上方setDutyCycle中的10000也改掉
    test_r.setDutyCycle(2000);//这里的占空比是低电位所占比例

    steer_test.enable();//右电机初始化 
    steer_test.setPeriod(20000000);

    motor_l.setDirection("out"); motor_l.setValue(0);//左电机方向脚初始化
    motor_r.setDirection("out"); motor_r.setValue(0);//右电机方向脚初始化




    Speed_decision();
    Data_Settings();
    InitMH();
    string command;

    /***********************************摄像头初始化******************************* */
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);  // 设置分辨率宽度为640
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240); // 设置分辨率高度为480
    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // 设置视频编码格式为MJPG

    cap.set(cv::CAP_PROP_FPS, 120); // 设置帧率为120 FPS

    // // // // 设置其他摄像头参数
    // cap.set(cv::CAP_PROP_BRIGHTNESS, 0);       // 亮度
    // cap.set(cv::CAP_PROP_CONTRAST, 32);        // 对比度
    // cap.set(cv::CAP_PROP_HUE, 0);              // 色调
    // cap.set(cv::CAP_PROP_SATURATION, 60);      // 饱和度
    // cap.set(cv::CAP_PROP_SHARPNESS, 2);        // 清晰度
    // cap.set(cv::CAP_PROP_GAMMA, 100);          // 伽马
    // cap.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 6500); // 白平衡
    // cap.set(cv::CAP_PROP_BACKLIGHT, 1);        // 逆光对比
    // cap.set(cv::CAP_PROP_GAIN, 0);             // 增益
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);   // 关闭自动曝光
    cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);   //再关闭自动曝光
    cap.set(cv::CAP_PROP_EXPOSURE, 156);        // 曝光

    // //关闭自动曝光
    // //cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    // cap.set(cv::CAP_PROP_AUTOFOCUS, 0);        // 自动对焦
    // cap.set(cv::CAP_PROP_ZOOM, 1);             // 缩放
    // cap.set(cv::CAP_PROP_FOCUS, 0);            // 对焦
    // cap.set(cv::CAP_PROP_TEMPERATURE, 6500);   // 色温
    // cap.set(cv::CAP_PROP_TRIGGER, 0);          // 触发
    // cap.set(cv::CAP_PROP_TRIGGER_DELAY, 0);    // 触发延迟
    // cap.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, 4600); // 红色白平衡
    // cap.set(cv::CAP_PROP_PAN, 0);              // 平移
    // cap.set(cv::CAP_PROP_TILT, 0);             // 倾斜
    // cap.set(cv::CAP_PROP_ROLL, 0);             // 滚动
    // cap.set(cv::CAP_PROP_IRIS, 0);             // 光圈
    // cap.set(cv::CAP_PROP_SETTINGS, 0);         // 设置
    // cap.set(cv::CAP_PROP_BUFFERSIZE, 10);       // 缓冲区大小
    // cap.set(cv::CAP_PROP_SAR_NUM, 1);          // 样本纵横比：分子
    // cap.set(cv::CAP_PROP_SAR_DEN, 1);          // 样本纵横比：分母
    // cap.set(cv::CAP_PROP_CHANNEL, 0);          // 视频输入或通道号
    // cap.set(cv::CAP_PROP_AUTO_WB, 0);          // 自动白平衡
    // cap.set(cv::CAP_PROP_WB_TEMPERATURE, 4600); // 白平衡色温
    /***************************************************************************** */
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
            printfshuzu();
        }



        imageprocess();//图像处理





        //test_l.disable();    
    }
    cap.release();
    return 0;
}







// int main()
// {
//     
//     double start_time = getTickCount();

//     VideoCapture cap(0);
//     cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
//     cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);  // 设置分辨率宽度为640
//     cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240); // 设置分辨率高度为480
//     cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G')); // 设置视频编码格式为MJPG
//     cap.set(cv::CAP_PROP_FPS, 120); // 设置帧率为120 FPS

//     //  // // // // 设置其他摄像头参数
//     //  cap.set(cv::CAP_PROP_BRIGHTNESS, 0);       // 亮度
//     //  cap.set(cv::CAP_PROP_CONTRAST, 32);        // 对比度
//     //  cap.set(cv::CAP_PROP_HUE, 0);              // 色调
//     //  cap.set(cv::CAP_PROP_SATURATION, 60);      // 饱和度
//     //  cap.set(cv::CAP_PROP_SHARPNESS, 2);        // 清晰度
//     //  cap.set(cv::CAP_PROP_GAMMA, 100);          // 伽马
//     //  cap.set(cv::CAP_PROP_WHITE_BALANCE_BLUE_U, 6500); // 白平衡
//     //  cap.set(cv::CAP_PROP_BACKLIGHT, 1);        // 逆光对比
//     //  cap.set(cv::CAP_PROP_GAIN, 0);             // 增益
//     //  // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 3);   // 关闭自动曝光
//     //  // cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);   //再关闭自动曝光
//     //  cap.set(cv::CAP_PROP_EXPOSURE, 156);        // 曝光

//      //关闭自动曝光
//      //cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
//     //  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);        // 自动对焦
//     //  cap.set(cv::CAP_PROP_ZOOM, 1);             // 缩放
//     //  cap.set(cv::CAP_PROP_FOCUS, 0);            // 对焦
//     //  cap.set(cv::CAP_PROP_TEMPERATURE, 6500);   // 色温
//     //  cap.set(cv::CAP_PROP_TRIGGER, 0);          // 触发
//     //  cap.set(cv::CAP_PROP_TRIGGER_DELAY, 0);    // 触发延迟
//     //  cap.set(cv::CAP_PROP_WHITE_BALANCE_RED_V, 4600); // 红色白平衡
//     //  cap.set(cv::CAP_PROP_PAN, 0);              // 平移
//     //  cap.set(cv::CAP_PROP_TILT, 0);             // 倾斜
//     //  cap.set(cv::CAP_PROP_ROLL, 0);             // 滚动
//     //  cap.set(cv::CAP_PROP_IRIS, 0);             // 光圈
//     //  cap.set(cv::CAP_PROP_SETTINGS, 0);         // 设置
//     //  cap.set(cv::CAP_PROP_BUFFERSIZE, 10);       // 缓冲区大小
//     //  cap.set(cv::CAP_PROP_SAR_NUM, 1);          // 样本纵横比：分子
//     //  cap.set(cv::CAP_PROP_SAR_DEN, 1);          // 样本纵横比：分母
//     //  cap.set(cv::CAP_PROP_CHANNEL, 0);          // 视频输入或通道号
//     //  cap.set(cv::CAP_PROP_AUTO_WB, 0);          // 自动白平衡
//     //  cap.set(cv::CAP_PROP_WB_TEMPERATURE, 4600); // 白平衡色温


//     // cap.set(CAP_PROP_FPS, 120);
//     // cap.set(CAP_PROP_FRAME_WIDTH, 640);
//     // cap.set(CAP_PROP_FRAME_HEIGHT, 480);
//     // cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M', 'J', 'P', 'G'));
//     // if (!cap.isOpened()){cout << "can not open camera" << endl;return -1;}

//     // PWM_ATIM test(86, 0b11, 3, 20000000, 1700,1);test.enable();test.setPeriod(20000000);//舵机初始化
//     // PWM_GTIM test_r(89, 0b11, 3, 10000, 0);test_r.enable();//右电机初始化
//     // PWM_GTIM test_l(88, 0b11, 2, 10000, 0);test_l.enable();//左电机初始化

//     // Data_Settings();
//     // string command;

//     while (true)
//     {
//         
//         //img_erzhi = erzhihua(img);
//         fps_count++;
//         if (fps_count % 10 == 0)// 每10帧计算一次FPS
//         {
//             double elapsed_time = (getTickCount() - start_time) / getTickFrequency();fps = fps_count / elapsed_time;
//             start_time = getTickCount();fps_count = 0;
//             cout<<fps<<endl;
//             //printfshuzu(); 
//         }

//         // imageprocess();//图像处理
//         // Motor_Control();//电机PWM转换
//         // SteerPID_Realize(ImageStatus.Det_True - ImageStatus.MiddleLine);//舵机PID计算

//         // test.setDutyCycle(PWM_STEER);//舵机PWM输出
//         // test_l.setDutyCycle(PWM_L_MOTOR);test_r.setDutyCycle(PWM_R_MOTOR);//电机PWM输出
//         // test_l.disable();
//     }

//     // 释放资源
//     return 0;
// }